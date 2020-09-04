/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <uart_dfu_cli.h>
#include <uart_dfu_util.h>

#include <zephyr.h>
#include <logging/log.h>
#include <sys/__assert.h>
#include <string.h>


/*****************************************************************************
* Structure definitions
*****************************************************************************/

struct fragment_in {
	atomic_ptr_t buf;
	size_t idx;
	size_t len;
};


/*****************************************************************************
* Static variables
*****************************************************************************/

static atomic_t api_avail		= ATOMIC_INIT(0);
static bool in_write			= false;
static int rx_opcode			= OPCODE_NONE;
static struct fragment_in fragment 	= {0};
static struct uart_dfu_cli_cb app_cb 	= {NULL};

LOG_MODULE_DECLARE(uart_dfu, CONFIG_UART_DFU_LIBRARY_LOG_LEVEL);


/*****************************************************************************
* Static functions
*****************************************************************************/

/*
 * General helper functions
 *****************************************************************************/

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

static enum uart_dfu_cli_err err_type_get(int err)
{
	switch (err) {
	case UART_DFU_CLI_SUCCESS:
	case UART_DFU_CLI_ERR_TIMEOUT:
	case UART_DFU_CLI_ERR_BREAK:
		return (enum uart_dfu_cli_err) err;
	default:
		return UART_DFU_CLI_ERR_FATAL;
	}
}

static void evt_send(enum uart_dfu_cli_evt_type type, enum uart_dfu_cli_err err)
{
	struct uart_dfu_cli_evt evt = {
		.type = type,
		.err = err
	};
	app_cb.evt_cb(&evt);
}

static int cli_send(const struct uart_dfu_pdu *pdu, size_t len)
{
	uint8_t *buf = uart_dfu_tx_buf_get();
	memcpy(buf, pdu, len);
	return uart_dfu_tx_start(len, CONFIG_UART_DFU_CLI_SEND_TIMEOUT);
}

static bool cli_writec_send(void)
{
	struct uart_dfu_hdr hdr;
	memset(&hdr, 0, sizeof(hdr));
	hdr.opcode = UART_DFU_OPCODE_WRITEC;

	size_t seg_size = fragment_seg_len();
	const uint8_t *seg_data = fragment_read(seg_size); /* FIXME: NULL?*/

	uint8_t *buf = uart_dfu_tx_buf_get();
	memcpy(&buf[0], &hdr, sizeof(hdr));
	memcpy(&buf[sizeof(hdr)], seg_data, seg_size);

	int err = uart_dfu_tx_start(sizeof(hdr) + seg_size,
				CONFIG_UART_DFU_CLI_SEND_TIMEOUT);
	__ASSERT(err == 0, "Unexpected UART error: %d.", err);

	return fragment_done();
}

static bool cli_write_seq_cont(void)
{
	if (fragment_done()) {
		fragment_init(NULL, 0);
		return false;
	}

	/* Continue transmitting the fragment */
	bool final = cli_writec_send();
	if (final) {
		/* Final segment - prepare to receive reply. */
		rx_opcode = UART_DFU_OPCODE_WRITEC;
		int err = uart_dfu_rx_start(
			PDU_SIZE(sizeof(struct uart_dfu_status_args)));
		__ASSERT(err == 0, "Unexpected UART error: %d.", err);
	}
	return true;
}


/*
 * Event handler functions
 *****************************************************************************/

static void cli_recv_status_handle(const struct uart_dfu_pdu *pdu)
{
	(void) atomic_set(&api_avail, 1);
	app_cb.status_cb(pdu->args.status.data.status);
}

static void cli_recv_writeh_handle(const struct uart_dfu_pdu *pdu)
{
	int status = pdu->args.status.data.status;

	if (status == 0) {
		in_write = true;
		(void) cli_write_seq_cont();
	} else {
		(void) atomic_set(&api_avail, 1);
		app_cb.status_cb(status);
	}
}

static void cli_recv_offset_handle(const struct uart_dfu_pdu *pdu)
{
	(void) atomic_set(&api_avail, 1);
	if (pdu->args.status.data.status < 0) {
		app_cb.status_cb(pdu->args.status.data.status);
	} else {
		app_cb.offset_cb(pdu->args.status.data.offset);
	}
}

static void cli_recv_handle(const uint8_t *buf, size_t len)
{
	if (len == 0) {
		return;
	}

	const struct uart_dfu_pdu *pdu = (const struct uart_dfu_pdu *) buf;
	int opcode = (int) pdu->hdr.opcode;

	if (len != PDU_SIZE(sizeof(struct uart_dfu_status_args))	||
	    !pdu->hdr.status 						||
	    rx_opcode != opcode) {
		/* Only accept status messages with the correct length
		   and expected opcode. */
		return;
	}
	
	/* Received expected message; stop timeout */
	rx_opcode = OPCODE_NONE;
	uart_dfu_rx_timeout_stop();

	switch (opcode) {
	case UART_DFU_OPCODE_INIT:
		LOG_DBG("cli: received INIT");
		cli_recv_status_handle(pdu);
		break;
	case UART_DFU_OPCODE_DONE:
		LOG_DBG("cli: received DONE");
		cli_recv_status_handle(pdu);
		break;
	case UART_DFU_OPCODE_WRITEC:
		LOG_DBG("cli: received WRITEC");
		cli_recv_status_handle(pdu);
		break;
	case UART_DFU_OPCODE_WRITEH:
		LOG_DBG("cli: received WRITEH");
		cli_recv_writeh_handle(pdu);
		break;
	case UART_DFU_OPCODE_OFFSET:
		LOG_DBG("cli: received OFFSET");
		cli_recv_offset_handle(pdu);
		break;
	default:
		break;
	}
}

static void cli_send_handle(void)
{
	if (in_write) {
		if (cli_write_seq_cont()) {
			return;
		} else {
			in_write = false;
		}
	}

	(void) uart_dfu_rx_timeout_start(
		CONFIG_UART_DFU_CLI_RESPONSE_TIMEOUT);
}

static void cli_recv_abort_handle(int err)
{
	LOG_DBG("Client receive aborted with err: %d", err);
	uart_dfu_sess_close(UART_DFU_SESS_CLI, err);
}

static void cli_send_abort_handle(int err)
{
	LOG_DBG("Client send aborted with err: %d.", err);
	uart_dfu_sess_close(UART_DFU_SESS_CLI, err);
}

static void cli_sess_enter_handle(void)
{
	state_reset();	
	(void) atomic_set(&api_avail, 1);

	LOG_DBG("Client session started.");
	evt_send(UART_DFU_CLI_EVT_STARTED, UART_DFU_CLI_SUCCESS);
}

static void cli_sess_exit_handle(int err)
{
	(void) atomic_set(&api_avail, 0);
	state_reset();

	LOG_DBG("Client session stopped.");
	evt_send(UART_DFU_CLI_EVT_STOPPED, err_type_get(err));
}


/*
 * API helper functions
 *****************************************************************************/

static int api_cli_send(struct uart_dfu_pdu *pdu, size_t len)
{
	int err;
	int opcode = pdu->hdr.opcode;

	if (!atomic_cas(&api_avail, 1, 0)) {
		return -EBUSY;
	}

	rx_opcode = opcode;
	err = uart_dfu_rx_start(PDU_SIZE(sizeof(struct uart_dfu_status_args)));
	if (err == 0) {
		err = cli_send(pdu, len);
		if (err != 0) {
			(void) uart_dfu_rx_stop();
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


/*****************************************************************************
* UART DFU API functions
*****************************************************************************/

void uart_dfu_cli_evt_handle(const struct uart_dfu_evt *const evt)
{
	switch (evt->type) {
	case UART_DFU_EVT_RX:
		cli_recv_handle(evt->data.rx.buf, evt->data.rx.len);
		break;
	case UART_DFU_EVT_RX_END:
		cli_recv_abort_handle(evt->data.err);
		break;
	case UART_DFU_EVT_TX_END:
		if (evt->data.err == 0) {
			cli_send_handle();
		} else {
			cli_send_abort_handle(evt->data.err);
		}
		break;
	case UART_DFU_EVT_SESS_ENTER:
		cli_sess_enter_handle();
		break;
	case UART_DFU_EVT_SESS_EXIT:
		cli_sess_exit_handle(evt->data.err);
		break;
	default:
		break;
	}
}


/*****************************************************************************
* API functions
*****************************************************************************/

int uart_dfu_cli_init(struct uart_dfu_cli_cb *callbacks)
{
	if (callbacks == NULL			||
	    callbacks->status_cb == NULL	||
	    callbacks->offset_cb == NULL	||
	    callbacks->evt_cb == NULL) {
		return -EINVAL;
	}

	fragment_init(NULL, 0);
	app_cb = *callbacks;
	return 0;
}


int uart_dfu_cli_init_send(size_t file_size)
{
	if (file_size > INT32_MAX) {
		return -ENOMEM;
	}

	struct uart_dfu_pdu pdu;
	memset(&pdu, 0, sizeof(pdu));
	pdu.hdr.opcode = UART_DFU_OPCODE_INIT;
	pdu.args.init.file_size = (uint32_t) file_size;

	return api_cli_send(&pdu, PDU_SIZE(sizeof(pdu.args.init)));
}

int uart_dfu_cli_write_send(const uint8_t *const buf, size_t len)
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

	struct uart_dfu_pdu pdu;
	memset(&pdu, 0, sizeof(pdu));
	pdu.hdr.opcode = UART_DFU_OPCODE_WRITEH;
	pdu.args.writeh.fragment_size = len;

	int err = api_cli_send(&pdu, PDU_SIZE(sizeof(pdu.args.writeh)));
	if (err != 0) {
		fragment_init(NULL, 0);
	}
	return err;
}

int uart_dfu_cli_offset_send(void)
{
	struct uart_dfu_pdu pdu;
	memset(&pdu, 0, sizeof(pdu));
	pdu.hdr.opcode = UART_DFU_OPCODE_OFFSET;

	return api_cli_send(&pdu, PDU_SIZE(sizeof(pdu.args.offset)));
}

int uart_dfu_cli_done_send(bool successful)
{
	struct uart_dfu_pdu pdu;
	memset(&pdu, 0, sizeof(pdu));
	pdu.hdr.opcode = UART_DFU_OPCODE_DONE;
	pdu.args.done.success = successful;

	return api_cli_send(&pdu, PDU_SIZE(sizeof(pdu.args.done)));
}

int uart_dfu_cli_start(void)
{
	int err = uart_dfu_sess_open(UART_DFU_SESS_CLI);
	if (err == -EALREADY) {
		return 0;
	} else {
		return err;
	}
}

int uart_dfu_cli_stop(void)
{
	return uart_dfu_sess_close(UART_DFU_SESS_CLI, UART_DFU_CLI_SUCCESS);
}
