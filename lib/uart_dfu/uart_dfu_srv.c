/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <uart_dfu_srv.h>
#include <uart_dfu_util.h>

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
* Static variables
*****************************************************************************/

static atomic_t in_session		= ATOMIC_INIT(0);
static atomic_t done_pending		= ATOMIC_INIT(0);
static atomic_t rx_state		= ATOMIC_INIT(OPCODE_NONE);
static struct fragment_out fragment 	= {0};
static struct uart_dfu_srv_cb app_cb 	= {NULL};

LOG_MODULE_DECLARE(uart_dfu, CONFIG_UART_DFU_LIBRARY_LOG_LEVEL);

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

static enum uart_dfu_srv_err err_type_get(int err)
{
	switch (err) {
	case UART_DFU_SRV_SUCCESS:
	case UART_DFU_SRV_ERR_ABORT:
	case UART_DFU_SRV_ERR_TIMEOUT:
	case UART_DFU_SRV_ERR_BREAK:
		return (enum uart_dfu_srv_err) err;
	default:
		return UART_DFU_SRV_ERR_FATAL;
	}
}

static void evt_send(enum uart_dfu_srv_evt_type type, enum uart_dfu_srv_err err)
{
	struct uart_dfu_srv_evt evt = {
		.type = type,
		.err = err
	};
	app_cb.evt_cb(&evt);
}

static int srv_send(struct uart_dfu_pdu *pdu, size_t len)
{
	uint8_t *buf = uart_dfu_tx_buf_get();
	memcpy(buf, pdu, len);
	return uart_dfu_tx_start(len, CONFIG_UART_DFU_SRV_SEND_TIMEOUT);
}

static bool srv_accept(int opcode, int rx_next)
{
	return atomic_cas(&rx_state, OPCODE_ANY, rx_next) ||
		atomic_cas(&rx_state, opcode, rx_next);
}

static uint32_t rx_size_get(void)
{
	atomic_val_t opcode = atomic_get(&rx_state);
	switch (opcode) {
	case UART_DFU_OPCODE_WRITEC:
		return sizeof(struct uart_dfu_hdr) + fragment_seg_len();
	default:
		return sizeof(struct uart_dfu_pdu);
	}
}


/*
 * Message handler functions
 *****************************************************************************/

/**
 * @brief Handle reception of UART_DFU_OPCODE_INIT
 */
static int srv_recv_init_handle(const struct uart_dfu_pdu *pdu,
				size_t len,
				struct uart_dfu_pdu *rsp)
{
	if (len != PDU_SIZE(sizeof(struct uart_dfu_init_args))) {
		return -ENOENT;
	}
	if (srv_accept(pdu->hdr.opcode, OPCODE_ANY)) {
		int err = uart_dfu_sess_open(UART_DFU_SESS_SRV);
		if (err != 0 && err != -EALREADY) {
			/* Session is busy. */
			return -ENOENT;
		}
		int status = app_cb.init_cb(pdu->args.init.file_size);
		rsp->args.status.data.status = status;
		return 0;
	} else {
		return -ENOENT;
	}
}

/**
 * @brief Handle reception of UART_DFU_OPCODE_WRITEH
 */
static int srv_recv_writeh_handle(const struct uart_dfu_pdu *pdu,
				  size_t len,
				  struct uart_dfu_pdu *rsp)
{
	if (len != PDU_SIZE(sizeof(struct uart_dfu_writeh_args))) {
		return -ENOENT;
	}
	if (srv_accept(pdu->hdr.opcode, UART_DFU_OPCODE_WRITEC)) {
		uint32_t fragment_len = pdu->args.writeh.fragment_size;
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
 * @brief Handle reception of UART_DFU_OPCODE_WRITEC
 */
static int srv_recv_writec_handle(const struct uart_dfu_pdu *pdu,
				  size_t len,
				  struct uart_dfu_pdu *rsp)
{
	size_t data_len = ARG_SIZE(len);

	if (data_len == 0 || !atomic_cas(&rx_state,
					UART_DFU_OPCODE_WRITEC,
					UART_DFU_OPCODE_WRITEC)) {
		return -ENOENT;
	}

	uart_dfu_rx_timeout_stop();
	int err = fragment_write(pdu->args.writec.data, data_len);
	if (err == 0) {
		/* Fragment done. */
		(void) atomic_set(&rx_state, OPCODE_ANY);
		int status = app_cb.write_cb(fragment.buf, fragment.len);
		fragment_reset();
		rsp->args.status.data.status = status;
		return 0;
	} else if (err == -EMSGSIZE) {
		/* Fragment not yet done. */
		return -ENOENT;
	} else {
		/* Fragment size too large. */
		(void) atomic_set(&rx_state, OPCODE_ANY);
		fragment_reset();
		return -ENOENT;
	}
}

/**
 * @brief Handle reception of UART_DFU_OPCODE_OFFSET
 */
static int srv_recv_offset_handle(const struct uart_dfu_pdu *pdu,
				  size_t len,
				  struct uart_dfu_pdu *rsp)
{
	if (len != PDU_SIZE(sizeof(struct uart_dfu_offset_args))) {
		return -ENOENT;
	}
	if (srv_accept(pdu->hdr.opcode, OPCODE_ANY)) {
		uart_dfu_rx_timeout_stop();
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
 * @brief Handle reception of UART_DFU_OPCODE_DONE
 */
static int srv_recv_done_handle(struct uart_dfu_pdu *pdu,
				size_t len,
				struct uart_dfu_pdu *rsp)
{
	if (len != PDU_SIZE(sizeof(struct uart_dfu_done_args))) {
		return -ENOENT;
	}
	if (srv_accept(pdu->hdr.opcode, OPCODE_ANY)) {
		uart_dfu_rx_timeout_stop();
		atomic_set(&done_pending, 1);
		(void) atomic_set(&rx_state, UART_DFU_OPCODE_INIT);
		int status = app_cb.done_cb((bool) pdu->args.done.success);
		rsp->args.status.data.status = status;
		return 0;
	} else {
		return -ENOENT;
	}
}

static void srv_recv_handle(struct uart_dfu_pdu *pdu, size_t len)
{
	int err;
	struct uart_dfu_pdu rsp;

	if (pdu->hdr.status || len == 0) {
		return;
	}

	memset(&rsp, 0, sizeof(rsp));
	switch (pdu->hdr.opcode) {
	case UART_DFU_OPCODE_INIT:
		LOG_DBG("srv: received INIT");
		err = srv_recv_init_handle(pdu, len, &rsp);
		break;
	case UART_DFU_OPCODE_WRITEH:
		LOG_DBG("srv: received WRITEH");
		err = srv_recv_writeh_handle(pdu, len, &rsp);
		break;
	case UART_DFU_OPCODE_WRITEC:
		LOG_DBG("srv: received WRITEC");
		err = srv_recv_writec_handle(pdu, len, &rsp);
		break;
	case UART_DFU_OPCODE_OFFSET:
		LOG_DBG("srv: received OFFSET");
		err = srv_recv_offset_handle(pdu, len, &rsp);
		break;
	case UART_DFU_OPCODE_DONE:
		LOG_DBG("srv: received DONE");
		err = srv_recv_done_handle(pdu, len, &rsp);
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
		err = srv_send(&rsp, sizeof(rsp));
		__ASSERT(err == 0, "Unexpected UART error: %d.", err);
	} else {
		(void) uart_dfu_rx_timeout_start(
			CONFIG_UART_DFU_SRV_RESPONSE_TIMEOUT);
	}

	/* Start next RX. */
	err = uart_dfu_rx_start(rx_size_get());
	__ASSERT(err == 0, "Unexpected UART error: %d.", err);
}

static void srv_send_handle(void)
{
	if (atomic_get(&done_pending)) {
		(void) uart_dfu_sess_close(UART_DFU_SESS_SRV,
					UART_DFU_SRV_SUCCESS);
	} else {
		(void) uart_dfu_rx_timeout_start(
			CONFIG_UART_DFU_SRV_RESPONSE_TIMEOUT);
	}
}

static void srv_recv_abort_handle(int err)
{
	(void) atomic_set(&rx_state, OPCODE_NONE);
	LOG_DBG("Server receive aborted with err: %d", err);
	(void) uart_dfu_sess_close(UART_DFU_SESS_SRV, err_type_get(err));
}

static void srv_send_abort_handle(int err)
{
	LOG_DBG("Server send aborted with err: %d", err);
	(void) uart_dfu_sess_close(UART_DFU_SESS_SRV, err_type_get(err));
}

static void srv_sess_enter_handle(void)
{
	(void) atomic_set(&done_pending, 0);
	(void) atomic_set(&in_session, 1);

	LOG_DBG("Server session started.");
	evt_send(UART_DFU_SRV_EVT_STARTED, UART_DFU_SRV_SUCCESS);
}

static void srv_sess_exit_handle(int err)
{
	(void) atomic_set(&done_pending, 0);
	(void) atomic_set(&in_session, 0);
	if (err != -ECONNABORTED) {
		(void) atomic_set(&rx_state, UART_DFU_OPCODE_INIT);
	} else {
		(void) atomic_set(&rx_state, OPCODE_NONE);
	}
	LOG_DBG("Server session stopped.");
	evt_send(UART_DFU_SRV_EVT_STOPPED, err_type_get(err));
}


/*****************************************************************************
* UART DFU API functions
*****************************************************************************/

void uart_dfu_srv_evt_handle(const struct uart_dfu_evt *const evt)
{
	switch (evt->type) {
	case UART_DFU_EVT_RX:
		srv_recv_handle(evt->data.rx.pdu, evt->data.rx.len);
		break;
	case UART_DFU_EVT_RX_END:
		srv_recv_abort_handle(evt->data.err);
		break;
	case UART_DFU_EVT_TX_END:
		if (evt->data.err == 0) {
			srv_send_handle();
		} else {
			srv_send_abort_handle(evt->data.err);
		}
		break;
	case UART_DFU_EVT_SESS_ENTER:
		srv_sess_enter_handle();
		break;
	case UART_DFU_EVT_SESS_EXIT:
		srv_sess_exit_handle(evt->data.err);
		break;
	default:
		break;
	}
}

void uart_dfu_srv_idle_evt_handle(const struct uart_dfu_evt *const evt)
{
	switch (evt->type) {
	case UART_DFU_EVT_RX:
		srv_recv_handle(evt->data.rx.pdu, evt->data.rx.len);
		break;
	case UART_DFU_EVT_RX_END:
		if (evt->data.err != -EFAULT) {
			(void) uart_dfu_rx_start(rx_size_get());
		}
		break;
	case UART_DFU_EVT_SESS_ENTER:
		(void) uart_dfu_rx_start(rx_size_get());
		break;
	case UART_DFU_EVT_SESS_EXIT:
	case UART_DFU_EVT_TX_END:
	default:
		break;
	}
}


/*****************************************************************************
* Public API functions
*****************************************************************************/

int uart_dfu_srv_init(uint8_t *fragment_buf,
		      size_t fragment_max_len,
		      struct uart_dfu_srv_cb *callbacks)
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
	return 0;
}

void uart_dfu_srv_enable(void)
{
	/* Accept INIT PDUs in the idle state. */
	(void) atomic_cas(&rx_state, OPCODE_NONE, UART_DFU_OPCODE_INIT);
}

int uart_dfu_srv_disable(void)
{
	if (uart_dfu_sess_close(UART_DFU_SESS_SRV,
				UART_DFU_SRV_ERR_ABORT) == -EFAULT) {
		atomic_set(&rx_state, OPCODE_NONE);
		return 0;
	} else {
		return -EINPROGRESS;
	}
}
