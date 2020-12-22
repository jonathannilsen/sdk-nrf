/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */


#include <zephyr.h>
#include <logging/log.h>
#include <sys/__assert.h>
#include <string.h>

#include <uart_blob_tx.h>
#include "uart_blob_util.h"


struct fragment_in {
	atomic_ptr_t buf;
	size_t idx;
	size_t len;
};

static void cobs_user_cb(const struct uart_cobs_user *user,
			 const struct uart_cobs_evt *evt);

UART_COBS_USER_DEFINE(cobs_user, cobs_user_cb);

static atomic_t api_avail;
static bool in_write;
static int rx_opcode = OPCODE_NONE;
static struct fragment_in fragment;
static struct uart_blob_tx_cb app_cb;

LOG_MODULE_REGISTER(uart_blob_tx, CONFIG_UART_BLOB_TX_LOG_LEVEL);


static void fragment_init(const uint8_t *buf, size_t len)
{
	atomic_ptr_set(&fragment.buf, (void *) buf);
	fragment.idx = 0;
	fragment.len = len;
}

static size_t fragment_seg_len(void)
{
	return seg_size_next(fragment.idx, fragment.len);
}

static const uint8_t *fragment_read(size_t len)
{
	const uint8_t *ptr;
	if (fragment.idx + len <= fragment.len) {
		ptr = (const uint8_t *) atomic_ptr_get(&fragment.buf);
		ptr = &ptr[fragment.idx];
		fragment.idx += len;
		return ptr;
	} else {
		return NULL;
	}
}

static bool fragment_done(void)
{
	return fragment.idx >= fragment.len;
}

static void state_reset(void)
{
	rx_opcode = OPCODE_NONE;
	in_write = false;
	fragment_init(NULL, 0);
}

static enum uart_blob_status status_type_get(int err)
{
	switch (err) {
	case UART_BLOB_SUCCESS:
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

static int pdu_send(const struct uart_blob_pdu *pdu, size_t len)
{
	int err = uart_cobs_tx_buf_write(&cobs_user, pdu, len);
	if (err) {
		return err;
	}
	err = uart_cobs_tx_start(&cobs_user, CONFIG_UART_BLOB_TX_SEND_TIMEOUT);
	if (err) {
		(void) uart_cobs_tx_buf_clear(&cobs_user);
	}
	return err;
}

static bool writec_send(void)
{
	struct uart_blob_hdr hdr;
	memset(&hdr, 0, sizeof(hdr));
	hdr.opcode = UART_BLOB_OPCODE_WRITEC;

	size_t seg_len = fragment_seg_len();
	const uint8_t *seg_data = fragment_read(seg_len);
	__ASSERT(seg_data != NULL, "Unexpected NULL value.");

	int err = uart_cobs_tx_buf_write(&cobs_user, &hdr, sizeof(hdr));
	__ASSERT(err == 0, "Unexpected error writing TX buffer: %d.", err);
	err = uart_cobs_tx_buf_write(&cobs_user, seg_data, seg_len);
	__ASSERT(err == 0, "Unexpected error writing TX buffer: %d.", err);

	int err = uart_cobs_tx_start(&cobs_user,
				     CONFIG_UART_BLOB_TX_SEND_TIMEOUT);
	__ASSERT(err == 0, "Unexpected UART error: %d.", err);

	return fragment_done();
}

static bool write_seq_cont(void)
{
	if (fragment_done()) {
		fragment_init(NULL, 0);
		return false;
	}

	/* Continue transmitting the fragment */
	bool final = writec_send();
	if (final) {
		/* Final segment - prepare to receive reply. */
		rx_opcode = UART_BLOB_OPCODE_WRITEC;
		int err = uart_cobs_rx_start(&cobs_user,
			PDU_SIZE(sizeof(struct uart_blob_status_args)));
		__ASSERT(err == 0, "Unexpected UART error: %d.", err);
	}
	return true;
}

static void init_recv_handle(const struct uart_blob_pdu *pdu)
{
	(void) atomic_set(&api_avail, 1);
	app_cb.init_cb(pdu->args.status.data.status);
}

static void writec_recv_handle(const struct uart_blob_pdu *pdu)
{
	(void) atomic_set(&api_avail, 1);
	app_cb.write_cb(pdu->args.status.data.status, ???);
}

static void done_recv_handle(const struct uart_blob_pdu *pdu)
{
	(void) atomic_set(&api_avail, 1);
	app_cb.done_cb(pdu->args.status.data.status);
}

static void writeh_recv_handle(const struct uart_blob_pdu *pdu)
{
	int status = pdu->args.status.data.status;
	if (status == 0) {
		in_write = true;
		(void) write_seq_cont();
	} else {
		(void) atomic_set(&api_avail, 1);
		app_cb.write_cb(status, 0);
	}
}

static void offset_recv_handle(const struct uart_blob_pdu *pdu)
{
	int status;
	size_t offset;

	(void) atomic_set(&api_avail, 1);
	if (pdu->args.status.data.status < 0) {
		status = pdu->args.status.data.status;
		offset = 0;
	} else {
		status = 0;
		offset = pdu->args.status.data.offset;
	}
	app_cb.offset_cb(status, offset);
}

static void recv_handle(const uint8_t *buf, size_t len)
{
	if (len == 0) {
		return;
	}

	const struct uart_blob_pdu *pdu = (const struct uart_blob_pdu *) buf;
	int opcode = (int) pdu->hdr.opcode;

	if (len != PDU_SIZE(sizeof(struct uart_blob_status_args))	||
	    !pdu->hdr.status 						||
	    rx_opcode != opcode) {
		/* Only accept status messages with the correct length
		   and expected opcode. */
		return;
	}
	
	/* Received expected message; stop timeout */
	rx_opcode = OPCODE_NONE;
	uart_cobs_rx_timeout_stop(&cobs_user);

	switch (opcode) {
	case UART_BLOB_OPCODE_INIT:
		init_recv_handle(pdu);
		break;
	case UART_BLOB_OPCODE_WRITEC:
		writec_recv_handle(pdu);
		break;
	case UART_BLOB_OPCODE_DONE:
		done_recv_handle(pdu);
		break;
	case UART_BLOB_OPCODE_WRITEH:
		writeh_recv_handle(pdu);
		break;
	case UART_BLOB_OPCODE_OFFSET:
		offset_recv_handle(pdu);
		break;
	default:
		break;
	}
}

static void send_handle(void)
{
	if (in_write) {
		if (write_seq_cont()) {
			return;
		} else {
			in_write = false;
		}
	}

	(void) uart_cobs_rx_timeout_start(&cobs_user,
					  CONFIG_UART_BLOB_TX_RESPONSE_TIMEOUT);
}

static void recv_abort_handle(enum uart_cobs_err err)
{
	(void) uart_cobs_user_end(cobs_user_cb, (int) err);
}

static void send_abort_handle(enum uart_cobs_err err)
{
	(void) uart_cobs_user_end(cobs_user_cb, (int) err);
}

static void user_start_handle(void)
{
	state_reset();	
	(void) atomic_set(&api_avail, 1);
	evt_send(UART_BLOB_EVT_STARTED, UART_BLOB_SUCCESS);
}

static void user_end_handle(int err)
{
	(void) atomic_set(&api_avail, 0);
	state_reset();
	evt_send(UART_BLOB_EVT_STOPPED, status_type_get(err));
}

static void cobs_user_cb(const struct uart_cobs_user *user,
			 const struct uart_cobs_evt *evt)
{
	ARG_UNUSED(user);

	switch (evt->type) {
	case UART_COBS_EVT_USER_START:
		user_start_handle();
		break;
	case UART_COBS_EVT_USER_END:
		user_end_handle(evt->data.err);
		break;
	case UART_COBS_EVT_RX:
		recv_handle(evt->data.rx.buf, evt->data.rx.len);
		break;
	case UART_COBS_EVT_RX_ERR:
		recv_abort_handle(evt->data.err);
		break;
	case UART_COBS_EVT_TX:
		send_handle();
		break;
	case UART_COBS_EVT_TX_ERR:
		send_abort_handle(evt->data.err);
		break;
	default:
		break;
	}
}

static int api_send(struct uart_blob_pdu *pdu, size_t len)
{
	if (!atomic_cas(&api_avail, 1, 0)) {
		return -EBUSY;
	}

	rx_opcode = pdu->hdr.opcode;
	int err = uart_cobs_rx_start(&cobs_user,
			PDU_SIZE(sizeof(struct uart_blob_status_args)));
	if (err == 0) {
		err = pdu_send(pdu, len);
		if (err != 0) {
			(void) uart_cobs_rx_stop(&cobs_user);
		}
	}

	if (err != 0) {
		rx_opcode = OPCODE_NONE;
		(void) atomic_set(&api_avail, 1);
		return -EBUSY;
	} else {
		return 0;
	}
}

int uart_blob_tx_init(struct uart_blob_tx_cb *callbacks)
{
	if (callbacks == NULL			||
	    callbacks->init_cb == NULL		||
	    callbacks->offset_cb == NULL	||
	    callbacks->write_cb == NULL		||
	    callbacks->done_cb == NULL		||
	    callbacks->evt_cb == NULL) {
		return -EINVAL;
	}

	fragment_init(NULL, 0);
	app_cb = *callbacks;
	return 0;
}

int uart_blob_tx_enable(void)
{
	int err = uart_cobs_user_start(&cobs_user);
	if (err == -EALREADY) {
		return 0;
	} else {
		return err;
	}
}

int uart_blob_tx_disable(void)
{
	return uart_cobs_user_end(&cobs_user, UART_BLOB_SUCCESS);
}

int uart_blob_tx_send_init(size_t blob_len)
{
	if (blob_len > INT32_MAX) {
		return -ENOMEM;
	}

	struct uart_blob_pdu pdu;
	memset(&pdu, 0, sizeof(pdu));
	pdu.hdr.opcode = UART_BLOB_OPCODE_INIT;
	pdu.args.init.blob_len = (uint32_t) blob_len;

	return api_send(&pdu, PDU_SIZE(sizeof(pdu.args.init)));
}

int uart_blob_tx_send_write(const uint8_t *const buf, size_t len)
{
	if (buf == NULL || len == 0) {
		return -EINVAL;
	}
	if (len > INT32_MAX) {
		return -ENOMEM;
	}
	if (!atomic_ptr_cas(&fragment.buf, NULL, (void *) buf)) {
		return -EBUSY;
	}
	fragment.idx = 0;
	fragment.len = len;

	struct uart_blob_pdu pdu;
	memset(&pdu, 0, sizeof(pdu));
	pdu.hdr.opcode = UART_BLOB_OPCODE_WRITEH;
	pdu.args.writeh.fragment_len = len;

	int err = api_send(&pdu, PDU_SIZE(sizeof(pdu.args.writeh)));
	if (err != 0) {
		fragment_init(NULL, 0);
	}
	return err;
}

int uart_blob_tx_send_offset(void)
{
	struct uart_blob_pdu pdu;
	memset(&pdu, 0, sizeof(pdu));
	pdu.hdr.opcode = UART_BLOB_OPCODE_OFFSET;

	return api_send(&pdu, PDU_SIZE(sizeof(pdu.args.offset)));
}

int uart_blob_tx_send_done(bool successful)
{
	struct uart_blob_pdu pdu;
	memset(&pdu, 0, sizeof(pdu));
	pdu.hdr.opcode = UART_BLOB_OPCODE_DONE;
	pdu.args.done.success = successful;

	return api_send(&pdu, PDU_SIZE(sizeof(pdu.args.done)));
}
