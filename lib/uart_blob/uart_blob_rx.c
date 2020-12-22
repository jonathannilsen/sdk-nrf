/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */


#include <zephyr.h>
#include <logging/log.h>
#include <sys/__assert.h>
#include <string.h>

#include <uart_blob_rx.h>
#include "uart_blob_util.h"


struct fragment_out {
	uint8_t *buf;
	size_t idx;
	size_t len;
	size_t max_len;
};


static void cobs_idle_cb(const struct uart_cobs_user *user,
			 const struct uart_cobs_evt *evt);
static void cobs_user_cb(const struct uart_cobs_user *user,
			 const struct uart_cobs_evt *evt);

UART_COBS_USER_DEFINE(idle_cobs_user, cobs_idle_cb);
UART_COBS_USER_DEFINE(cobs_user, cobs_user_cb);

static bool done_pending;
static int rx_state = OPCODE_NONE;
static struct fragment_out fragment;
static struct uart_blob_rx_cb app_cb;

LOG_MODULE_REGISTER(uart_blob_rx, CONFIG_UART_BLOB_RX_LOG_LEVEL);


static int fragment_prepare(size_t len)
{
	if (len <= fragment.max_len) {
		fragment.idx = 0;
		fragment.len = len;
		return 0;
	} else {
		return -ENOMEM;
	}
}

static void fragment_reset(void)
{
	fragment.idx = 0;
	fragment.len = 0;
}

static size_t fragment_seg_len(void)
{
	return seg_size_next(fragment.idx, fragment.len);
}

static int fragment_write(const uint8_t *data, size_t len)
{
	if (fragment.idx + len <= fragment.len) {
		memcpy(&fragment.buf[fragment.idx], data, len);
		fragment.idx += len;
		if (fragment.idx >= fragment.len) {
			/* Buffer filled */
			return 0;
		} else {
			/* Buffer not yet filled */
			return -EMSGSIZE;
		}
	} else {
		/* Not enough buffer space */
		return -ENOMEM;
	}
}

static enum uart_blob_status status_type_get(int err)
{
	switch (err) {
	case UART_BLOB_SUCCESS:
	case UART_BLOB_ERR_ABORT:
	case UART_BLOB_ERR_TIMEOUT:
	case UART_BLOB_ERR_BREAK:
		return (enum uart_blob_status) err;
	default:
		return UART_BLOB_ERR_FATAL;
	}
}

static void evt_send(enum uart_blob_evt_type type, enum uart_blob_status err)
{
	struct uart_blob_evt evt = {
		.type = type,
		.status = err
	};
	app_cb.evt_cb(&evt);
}

static int pdu_send(const struct uart_cobs_user *user,
		    const struct uart_blob_pdu *pdu, size_t len)
{
	int err = uart_cobs_tx_buf_write(user, pdu, len);
	if (err) {
		return err;
	}
	err = uart_cobs_tx_start(user, CONFIG_UART_BLOB_RX_SEND_TIMEOUT);
	if (err) {
		(void) uart_cobs_tx_buf_clear(user);
	}
	return err;
}

static bool opcode_accept(int opcode, int rx_next)
{
	if (rx_state == OPCODE_ANY || rx_state == opcode) {
		rx_state = rx_next;
		return true;
	} else {
		return false;
	}
}

static uint32_t rx_size_get(void)
{
	switch (rx_state) {
	case UART_BLOB_OPCODE_WRITEC:
		return sizeof(struct uart_blob_hdr) + fragment_seg_len();
	default:
		return sizeof(struct uart_blob_pdu);
	}
}

static void user_start_handle(void)
{
	done_pending = false;
	evt_send(UART_BLOB_EVT_STARTED, UART_BLOB_SUCCESS);
}

static void user_end_handle(enum uart_cobs_err err)
{
	done_pending = false;
	if (err != -ECONNABORTED) {
		rx_state = UART_BLOB_OPCODE_INIT;
	} else {
		rx_state = OPCODE_NONE;
	}
	evt_send(UART_BLOB_EVT_STOPPED, status_type_get(err));
}

static int init_recv_handle(const struct uart_blob_pdu *pdu,
			    size_t len,
			    struct uart_blob_pdu *rsp)
{
	if (len != PDU_SIZE(sizeof(struct uart_blob_init_args))) {
		return -ENOENT;
	}
	if (opcode_accept(pdu->hdr.opcode, OPCODE_ANY)) {
		int err = uart_cobs_user_start(cobs_user_cb);
		if (err != 0 && err != -EALREADY) {
			/* Busy. */
			return -ENOENT;
		}
		int status = app_cb.init_cb(pdu->args.init.blob_len);
		rsp->args.status.data.status = status;
		return 0;
	} else {
		return -ENOENT;
	}
}

static int writeh_recv_handle(const struct uart_blob_pdu *pdu,
			      size_t len,
			      struct uart_blob_pdu *rsp)
{
	if (len != PDU_SIZE(sizeof(struct uart_blob_writeh_args))) {
		return -ENOENT;
	}
	if (opcode_accept(pdu->hdr.opcode, UART_BLOB_OPCODE_WRITEC)) {
		uint32_t fragment_len = pdu->args.writeh.fragment_len;
		int err = fragment_prepare(fragment_len);
		if (err == -ENOMEM) {
			fragment_reset();
		}
		rsp->args.status.data.status = err;
		return 0;
	} else {
		return -ENOENT;
	}
}

static int writec_recv_handle(const struct uart_cobs_user *user,
			      const struct uart_blob_pdu *pdu,
			      size_t len,
			      struct uart_blob_pdu *rsp)
{
	size_t data_len = ARG_SIZE(len);

	if (data_len == 0 || rx_state != UART_BLOB_OPCODE_WRITEC) {
		return -ENOENT;
	}

	uart_cobs_rx_timeout_stop(user);
	int err = fragment_write(pdu->args.writec.data, data_len);
	if (err == 0) {
		/* Fragment done. */
		rx_state = OPCODE_ANY;
		int status = app_cb.write_cb(fragment.buf, fragment.len);
		fragment_reset();
		rsp->args.status.data.status = status;
		return 0;
	} else if (err == -EMSGSIZE) {
		/* Fragment not yet done. */
		return -ENOENT;
	} else {
		/* Fragment size too large. */
		rx_state = OPCODE_ANY;
		fragment_reset();
		return -ENOENT;
	}
}

static int offset_recv_handle(const struct uart_cobs_user *user,
			      const struct uart_blob_pdu *pdu,
			      size_t len,
			      struct uart_blob_pdu *rsp)
{
	if (len != PDU_SIZE(sizeof(struct uart_blob_offset_args))) {
		return -ENOENT;
	}
	if (opcode_accept(pdu->hdr.opcode, OPCODE_ANY)) {
		uart_cobs_rx_timeout_stop(user);
		size_t offset;
		int status = app_cb.offset_cb(&offset);
		if (status == 0) {
			if (offset <= OFFSET_MAX_ALLOWED) {
				rsp->args.status.data.offset = offset;
			} else {
				/* Invalid offset returned in callback */
				rsp->args.status.data.status = -EINVAL;
			}
		} else {
			rsp->args.status.data.status = status;
		}
		return 0;
	} else {
		return -ENOENT;
	}
}

static int done_recv_handle(const struct uart_cobs_user *user,
			    const struct uart_blob_pdu *pdu,
			    size_t len,
			    struct uart_blob_pdu *rsp)
{
	if (len != PDU_SIZE(sizeof(struct uart_blob_done_args))) {
		return -ENOENT;
	}
	if (opcode_accept(pdu->hdr.opcode, OPCODE_ANY)) {
		uart_cobs_rx_timeout_stop(user);
		done_pending = true;
		rx_state = UART_BLOB_OPCODE_INIT;
		int status = app_cb.done_cb((bool) pdu->args.done.success);
		rsp->args.status.data.status = status;
		return 0;
	} else {
		return -ENOENT;
	}
}

static void recv_handle(const struct uart_cobs_user *user,
			const uint8_t *buf, size_t len)
{
	const struct uart_blob_pdu *pdu = (const struct uart_blob_pdu *) buf;
	if (len == 0 || pdu->hdr.status) {
		/* Do not accept - RX will time out. */
		return;
	}

	int err;
	struct uart_blob_pdu rsp;
	memset(&rsp, 0, sizeof(rsp));
	
	switch (pdu->hdr.opcode) {
	case UART_BLOB_OPCODE_INIT:
		err = init_recv_handle(pdu, len, &rsp);
		break;
	case UART_BLOB_OPCODE_WRITEH:
		err = writeh_recv_handle(pdu, len, &rsp);
		break;
	case UART_BLOB_OPCODE_WRITEC:
		err = writec_recv_handle(pdu, len, &rsp);
		break;
	case UART_BLOB_OPCODE_OFFSET:
		err = offset_recv_handle(pdu, len, &rsp);
		break;
	case UART_BLOB_OPCODE_DONE:
		err = done_recv_handle(pdu, len, &rsp);
		break;
	default:
		LOG_ERR("Received unknown opcode %d", pdu->hdr.opcode);
		err = -ENOENT;
		break;
	}

	/* Send reply. */
	if (err == 0) {
		rsp.hdr.opcode = pdu->hdr.opcode;
		rsp.hdr.status = 1;
		err = pdu_send(user, &rsp, sizeof(rsp));
		__ASSERT(err == 0, "Unexpected UART error: %d.", err);
	} else {
		/* No response - set timeout for next RX. */
		(void) uart_cobs_rx_timeout_start(user,
			CONFIG_UART_BLOB_RX_RESPONSE_TIMEOUT);
	}

	/* Start next RX. */
	err = uart_cobs_rx_start(user, rx_size_get());
	__ASSERT(err == 0, "Unexpected UART error: %d.", err);
}

static void send_handle(const struct uart_cobs_user *user)
{
	if (done_pending) {
		(void) uart_cobs_user_end(user, UART_BLOB_SUCCESS);
	} else {
		(void) uart_cobs_rx_timeout_start(user,
			CONFIG_UART_BLOB_RX_RESPONSE_TIMEOUT);
	}
}

static void recv_abort_handle(const struct uart_cobs_user *user, int err)
{
	(void) uart_cobs_user_end(user, status_type_get(err));
}

static void send_abort_handle(const struct uart_cobs_user *user, int err)
{
	(void) uart_cobs_user_end(user, status_type_get(err));
}


static void cobs_user_cb(const struct uart_cobs_user *user,
			 const struct uart_cobs_evt *evt)
{
	switch (evt->type) {
	case UART_COBS_EVT_USER_START:
		user_start_handle();
		break;
	case UART_COBS_EVT_USER_END:
		user_end_handle(evt->data.end.status);
		break;
	case UART_COBS_EVT_RX:
		recv_handle(user, evt->data.rx.buf, evt->data.rx.len);
		break;
	case UART_COBS_EVT_RX_ERR:
		recv_abort_handle(user, evt->data.err);
		break;
	case UART_COBS_EVT_TX:
		send_handle(user);
		break;
	case UART_COBS_EVT_TX_ERR:
		send_abort_handle(user, evt->data.err);
		break;
	default:
		break;
	}
}

static void idle_recv_handle(const struct uart_cobs_user *user,
			     const uint8_t *buf, size_t len)
{
	const struct uart_blob_pdu *pdu = (const struct uart_blob_pdu *) buf;

	if (len > 0 && pdu->hdr.status &&
	    pdu->hdr.opcode == UART_BLOB_OPCODE_INIT) {
		struct uart_blob_pdu rsp;

		memset(&rsp, 0, sizeof(rsp));
		rsp.hdr.opcode = pdu->hdr.opcode;
		rsp.hdr.status = 1;
		if (init_recv_handle(pdu, len, &rsp) == 0) {
			int err = pdu_send(user, &rsp, sizeof(rsp));
			__ASSERT(err == 0, "Unexpected UART error: %d.", err);
			return;
		}
	}

	/* Start next RX. */
	int err = uart_cobs_rx_start(user, rx_size_get());
	__ASSERT(err == 0, "Unexpected UART error: %d.", err);
}

static void cobs_idle_cb(const struct uart_cobs_user *user,
			 const struct uart_cobs_evt *evt)
{
	switch (evt->type) {
	case UART_COBS_EVT_RX:
		idle_recv_handle(user, evt->data.rx.buf, evt->data.rx.len);
		break;
	case UART_COBS_EVT_USER_START:
	case UART_COBS_EVT_RX_ERR:
		(void) uart_cobs_rx_start(user, rx_size_get());
		break;
	case UART_COBS_EVT_USER_END:
	case UART_COBS_EVT_TX_ERR:
	default:
		break;
	}
}

int uart_blob_rx_init(uint8_t *fragment_buf,
		      size_t fragment_max_len,
		      struct uart_blob_rx_cb *callbacks)
{
	if (fragment_buf == NULL		||
	    fragment_max_len == 0		||
	    callbacks == NULL			||
	    callbacks->init_cb == NULL		||
	    callbacks->write_cb == NULL		||
	    callbacks->offset_cb == NULL	||
	    callbacks->done_cb == NULL		||
	    callbacks->evt_cb == NULL) {
		return -EINVAL;
	}

	memset(&fragment, 0, sizeof(fragment));
	fragment.buf = fragment_buf;
	fragment.max_len = fragment_max_len;
	app_cb = *callbacks;

	int err = uart_cobs_idle_user_set(cobs_idle_cb);
	if (err == -EBUSY) {
		return err;
	} else {
		return 0;
	}
}

void uart_blob_rx_enable(void)
{
	/* Accept INIT PDUs in the idle state. */
	if (rx_state == OPCODE_NONE) {
		rx_state = UART_BLOB_OPCODE_INIT;
	}
}

int uart_blob_rx_disable(void)
{
	int err = uart_cobs_user_end(cobs_user_cb, UART_BLOB_ERR_ABORT);
	if (err == -EBUSY) {
		rx_state = OPCODE_NONE;
		return 0;
	} else {
		return err;
	}
}
