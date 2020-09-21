/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <uart_blob_rx.h>
#include <uart_blob_util.h>

#include <zephyr.h>
#include <logging/log.h>
#include <sys/__assert.h>
#include <string.h>


/*****************************************************************************
* Structure definitions
*****************************************************************************/

struct fragment_out {
	uint8_t *buf;
	size_t idx;
	size_t len;
	size_t max_len;
};


/*****************************************************************************
* Forward declarations 
*****************************************************************************/

static void cobs_user_cb(const struct uart_cobs_evt *const evt)


/*****************************************************************************
* Static variables
*****************************************************************************/

static bool done_pending		= false;
static int rx_state			= OPCODE_NONE;
static struct fragment_out fragment 	= {0};
static struct uart_blob_rx_cb app_cb 	= {NULL};

LOG_MODULE_REGISTER(uart_blob_rx, CONFIG_UART_BLOB_RX_LOG_LEVEL);


/*****************************************************************************
* Static functions
*****************************************************************************/

/*
 * General helper functions
 *****************************************************************************/

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

static enum uart_blob_rx_err err_type_get(int err)
{
	switch (err) {
	case UART_BLOB_RX_SUCCESS:
	case UART_BLOB_RX_ERR_ABORT:
	case UART_BLOB_RX_ERR_TIMEOUT:
	case UART_BLOB_RX_ERR_BREAK:
		return (enum uart_blob_rx_err) err;
	default:
		return UART_BLOB_RX_ERR_FATAL;
	}
}

static void evt_send(enum uart_blob_rx_evt_type type, enum uart_blob_rx_err err)
{
	struct uart_blob_rx_evt evt = {
		.type = type,
		.err = err
	};
	app_cb.evt_cb(&evt);
}

static int pdu_send(struct uart_blob_pdu *pdu, size_t len)
{
	uint8_t *buf = uart_cobs_tx_buf_get();
	memcpy(buf, pdu, len);
	return uart_cobs_tx_start(len, CONFIG_UART_BLOB_RX_SEND_TIMEOUT);
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


/*
 * Message handler functions
 *****************************************************************************/

/**
 * @brief Handle reception of UART_BLOB_OPCODE_INIT
 */
static int init_recv_handle(const struct uart_blob_pdu *pdu,
				size_t len,
				struct uart_blob_pdu *rsp)
{
	if (len != PDU_SIZE(sizeof(struct uart_blob_init_args))) {
		return -ENOENT;
	}
	if (opcode_accept(pdu->hdr.opcode, OPCODE_ANY)) {
		int err = uart_cobs_user_request();
		if (err != 0 && err != -EALREADY) {
			/* Session is busy. */
			return -ENOENT;
		}
		int status = app_cb.init_cb(pdu->args.init.blob_len);
		rsp->args.status.data.status = status;
		return 0;
	} else {
		return -ENOENT;
	}
}

/**
 * @brief Handle reception of UART_BLOB_OPCODE_WRITEH
 */
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

/**
 * @brief Handle reception of UART_BLOB_OPCODE_WRITEC
 */
static int writec_recv_handle(const struct uart_blob_pdu *pdu,
				  size_t len,
				  struct uart_blob_pdu *rsp)
{
	size_t data_len = ARG_SIZE(len);

	if (data_len == 0 || rx_state != UART_BLOB_OPCODE_WRITEC) {
		return -ENOENT;
	}

	uart_cobs_rx_timeout_stop();
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

/**
 * @brief Handle reception of UART_BLOB_OPCODE_OFFSET
 */
static int offset_recv_handle(const struct uart_blob_pdu *pdu,
				  size_t len,
				  struct uart_blob_pdu *rsp)
{
	if (len != PDU_SIZE(sizeof(struct uart_blob_offset_args))) {
		return -ENOENT;
	}
	if (opcode_accept(pdu->hdr.opcode, OPCODE_ANY)) {
		uart_cobs_rx_timeout_stop();
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

/**
 * @brief Handle reception of UART_BLOB_OPCODE_DONE
 */
static int done_recv_handle(const struct uart_blob_pdu *pdu,
				size_t len,
				struct uart_blob_pdu *rsp)
{
	if (len != PDU_SIZE(sizeof(struct uart_blob_done_args))) {
		return -ENOENT;
	}
	if (opcode_accept(pdu->hdr.opcode, OPCODE_ANY)) {
		uart_cobs_rx_timeout_stop();
		done_pending = true;
		rx_state = UART_BLOB_OPCODE_INIT;
		int status = app_cb.done_cb((bool) pdu->args.done.success);
		rsp->args.status.data.status = status;
		return 0;
	} else {
		return -ENOENT;
	}
}

static void recv_handle(const uint8_t *buf, size_t len)
{
	const struct uart_blob_pdu *pdu = (const struct uart_blob_pdu *) buf;
	if (len == 0 || pdu->hdr.status) {
		return;
	}

	int err;
	struct uart_blob_pdu rsp;
	memset(&rsp, 0, sizeof(rsp));
	
	switch (pdu->hdr.opcode) {
	case UART_BLOB_OPCODE_INIT:
		LOG_DBG("srv: received INIT");
		err = init_recv_handle(pdu, len, &rsp);
		break;
	case UART_BLOB_OPCODE_WRITEH:
		LOG_DBG("srv: received WRITEH");
		err = writeh_recv_handle(pdu, len, &rsp);
		break;
	case UART_BLOB_OPCODE_WRITEC:
		LOG_DBG("srv: received WRITEC");
		err = writec_recv_handle(pdu, len, &rsp);
		break;
	case UART_BLOB_OPCODE_OFFSET:
		LOG_DBG("srv: received OFFSET");
		err = offset_recv_handle(pdu, len, &rsp);
		break;
	case UART_BLOB_OPCODE_DONE:
		LOG_DBG("srv: received DONE");
		err = done_recv_handle(pdu, len, &rsp);
		break;
	default:
		LOG_ERR("srv: received unknown opcode %d", pdu->hdr.opcode);
		err = -ENOENT;
		break;
	}

	/* Send reply. */
	if (err == 0) {
		rsp.hdr.opcode = pdu->hdr.opcode;
		rsp.hdr.status = 1;
		err = pdu_send(&rsp, sizeof(rsp));
		__ASSERT(err == 0, "Unexpected UART error: %d.", err);
	} else {
		(void) uart_cobs_rx_timeout_start(
			CONFIG_UART_BLOB_RX_RESPONSE_TIMEOUT);
	}

	/* Start next RX. */
	err = uart_cobs_rx_start(rx_size_get());
	__ASSERT(err == 0, "Unexpected UART error: %d.", err);
}

static void send_handle(void)
{
	if (done_pending) {
		(void) uart_cobs_user_release(cobs_user_cb,
					      UART_BLOB_RX_SUCCESS);
	} else {
		(void) uart_cobs_rx_timeout_start(
			CONFIG_UART_BLOB_RX_RESPONSE_TIMEOUT);
	}
}

static void recv_abort_handle(int err)
{
	LOG_DBG("Server receive aborted with err: %d", err);
	(void) uart_cobs_user_release(cobs_user_cb, err_type_get(err));
}

static void send_abort_handle(int err)
{
	LOG_DBG("Server send aborted with err: %d", err);
	(void) uart_cobs_user_release(cobs_user_cb, err_type_get(err));
}

static void user_start_handle(void)
{
	done_pending = false;
	LOG_DBG("Server session started.");
	evt_send(UART_BLOB_RX_EVT_STARTED, UART_BLOB_RX_SUCCESS);
}

static void user_end_handle(int err)
{
	done_pending = false;
	if (err != -ECONNABORTED) {
		rx_state = UART_BLOB_OPCODE_INIT;
	} else {
		rx_state = OPCODE_NONE;
	}
	LOG_DBG("Server session stopped.");
	evt_send(UART_BLOB_RX_EVT_STOPPED, err_type_get(err));
}

static void cobs_user_cb(const struct uart_cobs_evt *const evt)
{
	switch (evt->type) {
	case UART_COBS_EVT_RX:
		recv_handle(evt->data.rx.buf, evt->data.rx.len);
		break;
	case UART_COBS_EVT_RX_END:
		recv_abort_handle(evt->data.err);
		break;
	case UART_COBS_EVT_TX_END:
		if (evt->data.err == 0) {
			send_handle();
		} else {
			send_abort_handle(evt->data.err);
		}
		break;
	case UART_COBS_EVT_USER_START:
		user_start_handle();
		break;
	case UART_COBS_EVT_USER_END:
		user_end_handle(evt->data.err);
		break;
	default:
		break;
	}
}

static void cobs_idle_cb(const struct uart_cobs_evt *const evt)
{
	switch (evt->type) {
	case UART_COBS_EVT_RX:
		recv_handle(evt->data.rx.buf, evt->data.rx.len);
		break;
	case UART_COBS_EVT_RX_END:
		if (evt->data.err != -EFAULT) {
			(void) uart_cobs_rx_start(rx_size_get());
		}
		break;
	case UART_COBS_EVT_USER_START:
		(void) uart_cobs_rx_start(rx_size_get());
		break;
	case UART_COBS_EVT_USER_END:
	case UART_COBS_EVT_TX_END:
	default:
		break;
	}
}


/*****************************************************************************
* Public API functions
*****************************************************************************/

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
	int err = uart_cobs_user_release(cobs_user_cb, UART_BLOB_RX_ERR_ABORT);
	if (err == -EBUSY) {
		rx_state = OPCODE_NONE;
		return 0;
	} else {
		return err;
	}
}
