/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <sys/time_units.h>
#include <sys/atomic.h>
#include <logging/log.h>
#include <string.h>
#include <drivers/uart.h>
#include <uart_dfu.h>
#include <uart_dfu_types.h>
#include <cobs.h>


/*
        TODO:
		- Review the flow of state variables to check for invalid states
                - Make flow control work with PC
                - Nice logging
		- Some weird stuff happens when looping back from server to
		  client on the same instance.
*/

/*****************************************************************************
* Macros
*****************************************************************************/

/* Convenience macros */
#define PDU_HEADER_SIZE			(sizeof(struct uart_dfu_hdr))
#define PDU_ARG_SIZE			(sizeof(union uart_dfu_args))
#define PDU_BASE_SIZE			(sizeof(struct uart_dfu_pdu))

#define OFFSET_MAX_ALLOWED		INT32_MAX
#define OPCODE_ANY 			-1
#define OPCODE_NONE			-2
#define OPCODE_INVALID			-3

#define RX_WAIT_SIZE			1
#define RX_NORMAL_SIZE			(COBS_ENCODED_SIZE(PDU_BASE_SIZE))
#define RX_WRITE_SIZE(seg_size)		(COBS_ENCODED_SIZE(PDU_HEADER_SIZE + \
							   (seg_size)))
#define RX_FRAME_SIZE(curr_size)	(curr_size + 1)

#define RX_TIMEOUT_MARGIN		5000 /* ms */	
#define TX_TIMEOUT_MARGIN		500 /* ms */			

/* FIXME: Measure correct stack size. */
#define UART_DFU_STACK_SIZE		1024


/*****************************************************************************
* Forward declarations 
*****************************************************************************/

static void rx_timer_expired(struct k_timer *timer);


/*****************************************************************************
* Structure definitions
*****************************************************************************/

enum inst_type {
	TYPE_NONE = 0b00,
	TYPE_CLI = 0b01,
	TYPE_SRV = 0b10,
	TYPE_BOTH = 0b11
};

enum inst_flags {
	FLAG_RX_RDY = 0,
	FLAG_RX_TIMEOUT,
	FLAG_RX_ABORT,
	FLAG_TX_RDY,
	FLAG_TX_ACTIVE,
	FLAG_TX_TIMEOUT,
	FLAG_TX_ABORT
};

struct rx_rdy_info {
	struct k_work work;
	const u8_t *buf;
	size_t offset;
	size_t len;
};

struct tx_dis_info {
	struct k_work work;
	const u8_t *buf;
	size_t len;
};

struct uart_dfu {
	const char *const label;
	struct device *dev;
	u32_t baudrate;	

	atomic_t rx;
	atomic_t tx;

	u8_t rx_buf[COBS_MAX_BYTES];
	struct cobs_decoder decoder;
	u8_t rx_decoded_buf[COBS_MAX_DATA_BYTES];
	
	struct rx_rdy_info rx_rdy;
	struct k_work rx_dis_work;
	struct tx_dis_info tx_dis;

	struct k_work_q workqueue;
	struct z_thread_stack_element *stack_area;

	struct uart_dfu_cli *cli;
	struct uart_dfu_srv *srv;
};


/*****************************************************************************
* Static variables
*****************************************************************************/

/* NOTE: could be done in a more flexible way. */
#define __UART_DFU_INSTANCE_ENABLED(prop) \
	DT_HAS_CHOSEN(prop)

#define UART_DFU_INSTANCE_ENABLED(idx) \
	__UART_DFU_INSTANCE_ENABLED(uart_dfu_uart_inst_##idx)

#define __UART_DFU_INSTANCE_SUPPORTED(prop) \
	DT_NODE_HAS_COMPAT_STATUS(DT_CHOSEN(prop), nordic_nrf_uarte, okay)

#define UART_DFU_INSTANCE_SUPPORTED(idx) \
	__UART_DFU_INSTANCE_SUPPORTED(uart_dfu_uart_inst_##idx)

#define UART_DFU_INSTANCE_DEF(idx) \
	static const char state_##idx##_label[] = \
		DT_LABEL(DT_CHOSEN(uart_dfu_uart_inst_##idx)); \
	K_THREAD_STACK_DEFINE(state_##idx##_stack_area, UART_DFU_STACK_SIZE); \
	static struct uart_dfu state_##idx = { \
		.label = state_##idx##_label, \
		.stack_area = state_##idx##_stack_area \
	}

#if UART_DFU_INSTANCE_ENABLED(0)
#if UART_DFU_INSTANCE_SUPPORTED(0)
UART_DFU_INSTANCE_DEF(0);
#else
#error "UART DFU UART instance 0 is not enabled or is not supported."
#endif
#endif

#if UART_DFU_INSTANCE_ENABLED(1)
#if UART_DFU_INSTANCE_SUPPORTED(1)
UART_DFU_INSTANCE_DEF(1);
#else
#error "UART DFU UART instance 0 is not enabled or is not supported."
#endif
#endif

static struct uart_dfu *devices[] = {
#if UART_DFU_INSTANCE_ENABLED(0)
	&state_0,
#else
	NULL,
#endif
#if UART_DFU_INSTANCE_ENABLED(1)
	&state_1,
#else
	NULL
#endif
};

LOG_MODULE_REGISTER(uart_dfu, CONFIG_UART_DFU_LIBRARY_LOG_LEVEL);


/*****************************************************************************
* Static functions
*****************************************************************************/

/*
 * Utility functions
 *****************************************************************************/

static struct uart_dfu *dfu_device_get(size_t idx)
{
	if (idx < ARRAY_SIZE(devices)) {
		return devices[idx];
	} else {
		return NULL;
	}
}

static size_t seg_size_next(size_t curr_idx, size_t total_size)
{
	size_t remaining_size = total_size - curr_idx;
	return MIN(remaining_size, COBS_MAX_DATA_BYTES - PDU_HEADER_SIZE);
}

static size_t seg_size_in_next(struct uart_dfu_buf_in *buf_in)
{
	return seg_size_next(buf_in->idx, buf_in->size);
}

static size_t seg_size_out_next(struct uart_dfu_buf_out *buf_out)
{
	return seg_size_next(buf_out->idx, buf_out->size);
}

static void buf_in_cfg(struct uart_dfu_buf_in *buf_in,
		       const u8_t *buf,
		       size_t idx,
		       size_t size)
{
	buf_in->buf = buf;
	buf_in->idx = idx;
	buf_in->size = size;
}

static const u8_t *buf_in_read(struct uart_dfu_buf_in *buf_in, size_t size)
{
	const u8_t *ptr;

	if (buf_in->idx + size <= buf_in->size) {
		ptr = &buf_in->buf[buf_in->idx];
		buf_in->idx += size;
		return ptr;
	} else {
		return NULL;
	}
}

static bool buf_in_done(struct uart_dfu_buf_in *buf_in)
{
	return buf_in->idx >= buf_in->size;
}

static void buf_out_cfg(struct uart_dfu_buf_out *buf_in,
		        u8_t *buf,
		        size_t idx,
		        size_t size)
{
	buf_in->buf = buf;
	buf_in->idx = idx;
	buf_in->size = size;
}

static int buf_out_prepare(struct uart_dfu_buf_out *buf_out, size_t size)
{
	if (size <= buf_out->max_size) {
		buf_out->idx = 0;
		buf_out->size = size;
		return 0;
	} else {
		return -ENOMEM;
	}
}

static int buf_out_write(struct uart_dfu_buf_out *buf_out,
			 u8_t *data,
			 size_t size)
{
	if (buf_out->idx + size <= buf_out->size) {
		memcpy(&buf_out->buf[buf_out->idx], data, size);
		buf_out->idx += size;
		if (buf_out->idx >= buf_out->size) {
			/* Buffer filled */
			return 0;
		} else {
			/* Buffer not yet filled */
			return -EMSGSIZE;
		}
	} else {
		return -ENOMEM;
	}
}

static void buf_out_reset(struct uart_dfu_buf_out *buf_out)
{
	buf_out->idx = 0;
	buf_out->size = 0;
}

static inline int byte_duration_get(int baudrate, int bytes)
{
	return ceiling_fraction(bytes * 8 * MSEC_PER_SEC, baudrate);
}

static inline int timeout_get(struct uart_dfu *dev, int bytes, int margin)
{
	/* XXX: do we need to assert that the baudrate is within bounds? */
	int baudrate = (int) dev->baudrate;
	return byte_duration_get(baudrate, bytes) + margin;
}


/*
 * Instance helper functions 
 *****************************************************************************/

static void inst_error(struct uart_dfu_inst *inst, int err, void *ctx)
{
	if (inst->error_cb != NULL) {
		inst->error_cb(err, ctx);
	}
}

static void inst_init(struct uart_dfu_inst *inst,
		      u8_t *tx_buf,
		      size_t tx_buf_size,
		      uart_dfu_error_t error_cb)
{
	inst->flags = ATOMIC_INIT(0);
	(void) atomic_set(&inst->tx_state, OPCODE_NONE);
	(void) atomic_set(&inst->rx_state, OPCODE_NONE);
	inst->tx_buf = tx_buf;
	inst->tx_size = 0;
	inst->tx_buf_size = tx_buf_size;
	inst->timeout = ATOMIC_INIT(0);
	k_timer_init(&inst->timer, rx_timer_expired, NULL);
	k_timer_user_data_set(&inst->timer, NULL);
	inst->error_cb = error_cb;
}

static int inst_pdu_encode(struct uart_dfu_inst *inst,
		      	   struct uart_dfu_hdr *hdr,
		      	   union uart_dfu_args *args,
		      	   size_t arg_size)
{
	int err;
	COBS_ENCODER_DECLARE(encoder, inst->tx_buf);

	if (COBS_ENCODED_SIZE(PDU_HEADER_SIZE + arg_size) > inst->tx_buf_size) {
		return -ENOMEM;
	}

	/* Encode message header. */
	err = cobs_encode(&encoder, (u8_t *) hdr, PDU_HEADER_SIZE);
	if (err != 0) {
		return err;
	}

	/* Encode message arguments. */
	if (arg_size > 0) {
		err = cobs_encode(&encoder, (u8_t *) args, arg_size);
		if (err != 0) {
			return err;
		}
	}

	err = cobs_encode_finish(&encoder, &inst->tx_size);
	return err;
}

static int inst_abort_check(struct uart_dfu_inst *inst,
			    int abort_flag,
			    int timeout_flag)
{
	bool aborted = atomic_test_and_clear_bit(&inst->flags, abort_flag);
	bool timed_out = atomic_test_and_clear_bit(&inst->flags, timeout_flag);
	
	if (aborted) {
		return -ECANCELED;
	} else if (timed_out) {
		return -ETIMEDOUT;
	} else {
		return 0;
	}
}

static int inst_rx_timeout_get(struct uart_dfu_inst *inst, int bytes)
{
	struct uart_dfu *dev = (struct uart_dfu *) inst->mod_ctx;
	return timeout_get(dev, bytes, RX_TIMEOUT_MARGIN);
}

static int inst_tx_timeout_get(struct uart_dfu_inst *inst, int bytes)
{
	struct uart_dfu *dev = (struct uart_dfu *) inst->mod_ctx;
	return timeout_get(dev, bytes, TX_TIMEOUT_MARGIN);
}

static bool inst_tx_rdy(struct uart_dfu_inst *inst)
{
	return atomic_test_bit(&inst->flags, FLAG_TX_RDY);
}

static int inst_tx_start(struct uart_dfu_inst *inst)
{
	int err;
	size_t tx_timeout;
	struct uart_dfu *dev = (struct uart_dfu *) inst->mod_ctx;

	tx_timeout = inst_tx_timeout_get(inst, inst->tx_size);
	LOG_DBG("%s enabling TX (size=%u, timeout=%d)",
		dev->label,
		inst->tx_size,
		tx_timeout);

	atomic_set_bit(&inst->flags, FLAG_TX_ACTIVE);
	err = uart_tx(dev->dev, inst->tx_buf, inst->tx_size, tx_timeout);
	if (err != 0) {
		atomic_clear_bit(&inst->flags, FLAG_TX_ACTIVE);
		LOG_ERR("%s error enabling TX: %d", dev->label, err);
	}
	return err;
}

static void inst_tx_finish(struct uart_dfu_inst *inst, bool timeout)
{
	if (timeout) {
		atomic_set_bit(&inst->flags, FLAG_TX_TIMEOUT);
	}
	atomic_clear_bit(&inst->flags, FLAG_TX_RDY);
	atomic_clear_bit(&inst->flags, FLAG_TX_ACTIVE);
}

static bool inst_tx_cancel(struct uart_dfu_inst *inst)
{
	struct uart_dfu *dev = (struct uart_dfu *) inst->mod_ctx;
	if (atomic_test_bit(&inst->flags, FLAG_TX_ACTIVE)) {
		/* Instance is transmitting, cancel */
		atomic_set_bit(&inst->flags, FLAG_TX_ABORT);
		(void) uart_tx_abort(dev->dev);
		return true;
	} else {
		/* Instance is not transmitting, clear and return */
		atomic_clear_bit(&inst->flags, FLAG_TX_RDY);
		return false;
	}
}

static int inst_tx_abort_check(struct uart_dfu_inst *inst)
{
	return inst_abort_check(inst, FLAG_TX_ABORT, FLAG_TX_TIMEOUT);
}

static bool inst_rx_rdy(struct uart_dfu_inst *inst)
{
	return atomic_test_bit(&inst->flags, FLAG_RX_RDY);
}

static int inst_rx_abort_check(struct uart_dfu_inst *inst)
{
	return inst_abort_check(inst, FLAG_RX_ABORT, FLAG_RX_TIMEOUT);
}

static bool inst_rx_cancel(struct uart_dfu_inst *inst)
{
	struct uart_dfu *dev = (struct uart_dfu *) inst->mod_ctx;
	if (atomic_test_bit(&inst->flags, FLAG_RX_RDY)) {
		/* Instance is receiving, cancel */
		atomic_set_bit(&inst->flags, FLAG_RX_ABORT);
		(void) uart_rx_disable(dev->dev);
		return true;
	}
	return false;
}

static void inst_rx_timeout(struct uart_dfu_inst *inst)
{
	struct uart_dfu *dev = (struct uart_dfu *) inst->mod_ctx;
	if (atomic_test_bit(&inst->flags, FLAG_RX_RDY)) {
		(void) atomic_set_bit(&inst->flags, FLAG_RX_TIMEOUT);
		(void) uart_rx_disable(dev->dev);
	}
}

static bool inst_rx_timeout_start(struct uart_dfu_inst *inst)
{
	int timeout;
	timeout = (int) atomic_set(&inst->timeout, 0);
	if (timeout > 0) {
		LOG_DBG("Starting timeout: %d ms", timeout);
		k_timer_start(&inst->timer, K_MSEC(timeout), K_NO_WAIT);
		return true;
	} else {
		return false;
	}
}

static void inst_rx_timeout_stop(struct uart_dfu_inst *inst)
{
	k_timer_stop(&inst->timer);
	atomic_clear_bit(&inst->flags, FLAG_RX_TIMEOUT);
}

static int inst_send_reserve(struct uart_dfu_inst *inst,
			     atomic_val_t tx_old,
			     atomic_val_t tx_new)
{
	if (tx_old == OPCODE_ANY && !inst_tx_rdy(inst)) {
		(void) atomic_set(&inst->tx_state, tx_new);
		return 0;
	} else if (!inst_tx_rdy(inst) &&
		   atomic_cas(&inst->tx_state, tx_old, tx_new)) {
		return 0;
	} else {
		return -EBUSY;
	}
}

static void inst_send_unreserve(struct uart_dfu_inst *inst)
{
	(void) atomic_set(&inst->tx_state, OPCODE_NONE);
}

static int inst_send_prepare(struct uart_dfu_inst *inst,
			     struct uart_dfu_hdr *hdr,
		    	     union uart_dfu_args *args,
		    	     size_t arg_size)
{
	int err;

	err = inst_pdu_encode(inst, hdr, args, arg_size);
	if (err != 0) {
		return -EINVAL;
	}
	atomic_set_bit(&inst->flags, FLAG_TX_RDY);
	return 0;
}

static int inst_send_cond(struct uart_dfu_inst *inst,
			  int opcode_from,
			  struct uart_dfu_hdr *hdr,
			  union uart_dfu_args *args,
			  size_t arg_size)
{
	int err;

	err = inst_send_reserve(inst, opcode_from, hdr->opcode);
	if (err != 0) {
		return -ECANCELED;
	}
	
	err = inst_send_prepare(inst, hdr, args, arg_size);
	if (err != 0) {
		inst_send_unreserve(inst);
		return -ECANCELED;
	}

	return 0;
}

static int inst_send(struct uart_dfu_inst *inst,
		     struct uart_dfu_hdr *hdr,
		     union uart_dfu_args *args,
		     size_t arg_size)
{
	return inst_send_cond(inst, OPCODE_ANY, hdr, args, arg_size);
}

static int inst_send_finish(struct uart_dfu_inst *inst, int opcode_to)
{
	return atomic_set(&inst->tx_state, opcode_to);
}

static bool inst_send_active(struct uart_dfu_inst *inst)
{
	return atomic_get(&inst->tx_state) != OPCODE_NONE;
}

static int inst_send_state_get(struct uart_dfu_inst *inst)
{
	return atomic_get(&inst->tx_state);
}

static int inst_send_cancel(struct uart_dfu_inst *inst)
{
	if (!inst_tx_cancel(inst)) {
		inst_send_finish(inst, OPCODE_NONE);
		return 0;
	}
	return -EINPROGRESS;
}

static int inst_recv_cond(struct uart_dfu_inst *inst, int rx_old, int rx_new)
{
	if (rx_old == OPCODE_ANY) {
		/* Unconditional recv */
		atomic_set(&inst->rx_state, rx_new);
		atomic_set_bit(&inst->flags, FLAG_RX_RDY);
		return 0;
	} else if (atomic_cas(&inst->rx_state, rx_old, rx_new)) {
		/* Conditional recv success */
		atomic_set_bit(&inst->flags, FLAG_RX_RDY);
		return 0;
	} else {
		/* Conditional recv failure */
		return -ECANCELED;
	}
}

static void inst_recv(struct uart_dfu_inst *inst, int rx_new)
{
	(void) inst_recv_cond(inst, OPCODE_ANY, rx_new);
}

static int inst_recv_finish_cond(struct uart_dfu_inst *inst,
				 int rx_old,
				 int rx_new)
{
	bool success;
	if (rx_old == OPCODE_ANY) {
		/* Unconditional recv finish */
		(void) atomic_set(&inst->rx_state, rx_new);
		success = true;
	} else {
		success = atomic_cas(&inst->rx_state, rx_old, rx_new);
	}
	if (success) {
		if (rx_new == OPCODE_NONE) {
			atomic_clear_bit(&inst->flags, FLAG_RX_RDY);
		}
		inst_rx_timeout_stop(inst);	
		return 0;
	} else {
		return -ENOTSUP;
	}
}

static void inst_recv_finish(struct uart_dfu_inst *inst, int rx_new)
{
	(void) inst_recv_finish_cond(inst, OPCODE_ANY, rx_new);
}

static bool inst_recv_active(struct uart_dfu_inst *inst)
{
	return atomic_get(&inst->rx_state) != OPCODE_NONE;
}

static int inst_recv_cancel(struct uart_dfu_inst *inst)
{
	if (!inst_rx_cancel(inst)) {
		(void) inst_recv_finish(inst, OPCODE_NONE);
		return 0;
	}
	return -EINPROGRESS;
}

static void inst_recv_timeout_set(struct uart_dfu_inst *inst, int timeout)
{
	(void) atomic_set(&inst->timeout, timeout);
}

static void inst_recv_timeout_clr(struct uart_dfu_inst *inst)
{
	k_timer_stop(&inst->timer);
	atomic_clear_bit(&inst->flags, FLAG_RX_TIMEOUT);
}

static int inst_active_abort(struct uart_dfu_inst *inst)
{
	bool done = true; 
	if (inst_send_active(inst)) {
		done = done && inst_send_cancel(inst);
	}
	if (inst_recv_active(inst)) {
		done = done && inst_recv_cancel(inst);
	}
	if (done) {
		return 0;
	} else {
		return -EINPROGRESS;
	}
}



/*
 * Client instance helper functions 
 *****************************************************************************/

static u32_t cli_rx_size_get(struct uart_dfu_cli *cli)
{
	if (cli == NULL || !inst_rx_rdy(&cli->inst)) {
		return 0;
	}

	return RX_NORMAL_SIZE;
}

static void cli_rsp_timeout_set(struct uart_dfu_cli *cli)
{
	int timeout;
	if (inst_recv_active(&cli->inst)) {
		timeout = inst_rx_timeout_get(&cli->inst, RX_NORMAL_SIZE);
		inst_recv_timeout_set(&cli->inst, timeout);
	}
}

static int cli_writec_send(struct uart_dfu_cli *cli, bool *final)
{
	size_t seg_size;
	struct uart_dfu_hdr hdr;
	const u8_t *seg_data;

	memset(&hdr, 0, PDU_HEADER_SIZE);
	hdr.opcode = UART_DFU_OPCODE_WRITEC;
	seg_size = seg_size_in_next(&cli->fragment);
	seg_data =  buf_in_read(&cli->fragment, seg_size);
	*final = buf_in_done(&cli->fragment);
	if (seg_data == NULL) {
		/* TODO: state? */
		return -ECANCELED;
	}
	return inst_send(&cli->inst,
			 &hdr,
			 (union uart_dfu_args *) seg_data,
			 seg_size);
}

static int cli_write_seq_cont(struct uart_dfu_cli *cli)
{
	int err;
	bool final = false;

	if (buf_in_done(&cli->fragment)) {
		/* Fragment transmission done */
		buf_in_cfg(&cli->fragment, NULL, 0, 0);
		inst_send_unreserve(&cli->inst);
		cli_rsp_timeout_set(cli);	
		return 0;
	}
	
	/* Continue transmitting the fragment */
	err = cli_writec_send(cli, &final);
	if (err < 0) {
		buf_in_cfg(&cli->fragment, NULL, 0, 0);
		return -ECANCELED;
	}
	if (final) {
		/* Final segment - prepare to receive reply. */
		inst_recv(&cli->inst, UART_DFU_OPCODE_WRITEC);
	}
	return 0;
}

static void cli_recv_status_handle(struct uart_dfu_cli *cli,
				   struct uart_dfu_pdu *pdu)
{
	cli->cb.status_cb(pdu->args.status.data.status, cli->ctx);
}

static void cli_recv_writeh_handle(struct uart_dfu_cli *cli,
				  struct uart_dfu_pdu *pdu)
{
	int err;
	if (pdu->args.status.data.status == 0) {
		err = cli_write_seq_cont(cli);
		if (err != 0) {
			inst_error(&cli->inst, err, cli->ctx);
		}
	} else {
		cli->cb.status_cb(pdu->args.status.data.status, cli->ctx);
	}
}

static void cli_recv_offset_handle(struct uart_dfu_cli *cli,
				   struct uart_dfu_pdu *pdu)
{
	if (pdu->args.status.data.status < 0) {
		cli->cb.status_cb(pdu->args.status.data.status, cli->ctx);
	} else {
		cli->cb.offset_cb(pdu->args.status.data.offset, cli->ctx);
	}
}

static void cli_recv_handle(struct uart_dfu *dev,
			    struct uart_dfu_pdu *pdu,
			    size_t len)
{
	int err;
	struct uart_dfu_cli *cli = dev->cli;
	int opcode = (int) pdu->hdr.opcode;

	ARG_UNUSED(len);

	if (cli == NULL) {
		return;
	}

	err = inst_recv_finish_cond(&cli->inst, opcode, OPCODE_NONE);
	if (err != 0) {
		/* Did not receive expected opcode. */
		return;
	}
	
	switch (opcode) {
	case UART_DFU_OPCODE_INIT:
	case UART_DFU_OPCODE_DONE:
	case UART_DFU_OPCODE_WRITEC:
		cli_recv_status_handle(cli, pdu);
		break;
	case UART_DFU_OPCODE_WRITEH:
		cli_recv_writeh_handle(cli, pdu);
		break;
	case UART_DFU_OPCODE_OFFSET:
		cli_recv_offset_handle(cli, pdu);
		break;
	default:
		/* Unknown opcode (this should not happen since the opcode is
		   checked earlier). */
		return;
	}
}

static void cli_recv_abort_handle(struct uart_dfu *dev, int err)
{
	struct uart_dfu_cli *cli = dev->cli;
	inst_recv_finish(&cli->inst, OPCODE_NONE);
	inst_error(&cli->inst, err, cli->ctx);
}

static void cli_send_handle(struct uart_dfu *dev)
{
	int err;
	struct uart_dfu_cli *cli = dev->cli;
	int opcode;

	opcode = inst_send_state_get(&cli->inst);
	if (opcode == UART_DFU_OPCODE_WRITEH) {
		(void) inst_send_finish(&cli->inst, UART_DFU_OPCODE_WRITEC);
	}
	if (opcode == UART_DFU_OPCODE_WRITEC) {
		err = cli_write_seq_cont(cli);
		if (err != 0) {
			/* NOTE: cli_write_seq_cont clears the TX
						state in case of errors. */
			inst_error(&cli->inst, err, cli->ctx);
			return;
		}
	} else {
		(void) inst_send_finish(&cli->inst, OPCODE_NONE);
		cli_rsp_timeout_set(cli);	
	}
}

static void cli_send_abort_handle(struct uart_dfu *dev, int err)
{
	struct uart_dfu_cli *cli = dev->cli;
	inst_send_finish(&cli->inst, OPCODE_NONE);
	inst_error(&cli->inst, err, cli->ctx);
}


/*
 * Server instance helper functions 
 *****************************************************************************/

static u32_t srv_rx_size_get(struct uart_dfu_srv *srv)
{
	atomic_val_t opcode;

	if (srv == NULL || !inst_rx_rdy(&srv->inst)) {
		return 0;
	}

	opcode = atomic_get(&srv->inst.rx_state);
	switch (opcode) {
	case UART_DFU_OPCODE_WRITEC: {
		size_t seg_size = seg_size_out_next(&srv->fragment);
		return (u32_t) RX_WRITE_SIZE(seg_size);
	}
	default: {
		return RX_NORMAL_SIZE;
	}
	}
}

static void srv_seg_timeout_set(struct uart_dfu_srv *srv)
{
	size_t seg_size = seg_size_out_next(&srv->fragment);
	int timeout = inst_rx_timeout_get(&srv->inst, RX_WRITE_SIZE(seg_size));
	inst_recv_timeout_set(&srv->inst, timeout);
}

static int srv_recv_init_handle(struct uart_dfu_srv *srv,
				struct uart_dfu_pdu *pdu,
				struct uart_dfu_pdu *rsp)
{
	inst_recv_finish(&srv->inst, OPCODE_ANY);
	int status = srv->cb.init_cb(pdu->args.init.file_size, srv->ctx);
	rsp->args.status.data.status = status;
	return 0;
}

static int srv_recv_writeh_handle(struct uart_dfu_srv *srv,
				  struct uart_dfu_pdu *pdu,
				  struct uart_dfu_pdu *rsp)
{
	int err;
	u32_t fragment_size = pdu->args.writeh.fragment_size;

	inst_recv_finish(&srv->inst, UART_DFU_OPCODE_WRITEC);
	err = buf_out_prepare(&srv->fragment, fragment_size);
	if (err == 0) {
		srv_seg_timeout_set(srv);
	} else if (err == -ENOMEM) {
		buf_out_reset(&srv->fragment);
	}
	rsp->args.status.data.status = err;
	return 0;
}

static int srv_recv_writec_handle(struct uart_dfu_srv *srv,
				  struct uart_dfu_pdu *pdu,
				  size_t len,
				  struct uart_dfu_pdu *rsp)
{
	int err;
	size_t data_len = len - PDU_HEADER_SIZE;


	err = buf_out_write(&srv->fragment, pdu->args.writec.data, data_len);
	if (err == 0) {
		/* Fragment done. */
		inst_recv_finish(&srv->inst, OPCODE_ANY);
		int status = srv->cb.write_cb(srv->fragment.buf,
					      srv->fragment.size,
					      srv->ctx);
		rsp->args.status.data.status = status;
		return 0;
	} else if (err == -EMSGSIZE) {
		/* Fragment not yet done. */
		inst_recv_finish(&srv->inst, UART_DFU_OPCODE_WRITEC);
		srv_seg_timeout_set(srv);
	} else if (err == -ENOMEM) {
		/* Fragment size too large. */
		inst_recv_finish(&srv->inst, OPCODE_ANY);
		buf_out_reset(&srv->fragment);
	}
	return -ENOENT;
}

static int srv_recv_offset_handle(struct uart_dfu_srv *srv,
				  struct uart_dfu_pdu *rsp)
{
	size_t offset;
	
	(void) inst_recv_finish(&srv->inst, OPCODE_ANY);
	int status = srv->cb.offset_cb(&offset, srv->ctx);
	if (status == 0) {
		if (offset <= OFFSET_MAX_ALLOWED) {
			rsp->args.status.data.offset = offset;
		} else {
			inst_error(&srv->inst, -EINVAL, srv->ctx);
			return -ENOENT;
		}
	} else {
		rsp->args.status.data.status = status;
	}
	return 0;
}

static int srv_recv_done_handle(struct uart_dfu_srv *srv,
				struct uart_dfu_pdu *pdu,
				struct uart_dfu_pdu *rsp)
{
	(void) inst_recv_finish(&srv->inst, OPCODE_ANY);
	int status = srv->cb.done_cb((bool) pdu->args.done.success, srv->ctx);
	rsp->args.status.data.status = status;
	return 0;
}

static void srv_recv_handle(struct uart_dfu *dev,
			   struct uart_dfu_pdu *pdu,
			   size_t len)
{
	int err;
	struct uart_dfu_srv *srv = dev->srv;
	struct uart_dfu_pdu rsp;

	if (srv == NULL) {
		return;
	}

	memset(&rsp.hdr, 0, PDU_HEADER_SIZE);
	memset(&rsp.args.status, 0, sizeof(rsp.args.status));

	switch (pdu->hdr.opcode) {
	case UART_DFU_OPCODE_INIT:
		err = srv_recv_init_handle(srv, pdu, &rsp);
		break;
	case UART_DFU_OPCODE_WRITEH:
		err = srv_recv_writeh_handle(srv, pdu, &rsp);
		break;
	case UART_DFU_OPCODE_WRITEC:
		err = srv_recv_writec_handle(srv, pdu, len, &rsp);
		break;
	case UART_DFU_OPCODE_OFFSET:
		err = srv_recv_offset_handle(srv, &rsp);
		break;
	case UART_DFU_OPCODE_DONE:
		err = srv_recv_done_handle(srv, pdu, &rsp);
		break;
	default:
		return;
	}

	if (err == 0) {
		rsp.hdr.opcode = pdu->hdr.opcode;
		rsp.hdr.status = 1;

		/* Send reply. */
		err = inst_send(&srv->inst,
				&rsp.hdr,
				&rsp.args,
				sizeof(rsp.args.status));
		if (err != 0) {
			LOG_ERR("%s srv_send: error %d", dev->label, err);
			inst_error(&srv->inst, -ECANCELED, srv->ctx);
		}	
	}
}

static void srv_recv_abort_handle(struct uart_dfu *dev, int err)
{
	struct uart_dfu_srv *srv = dev->srv;
	if (err == -ECANCELED) {
		/* Abort - RX should stop */
		inst_recv_finish(&srv->inst, OPCODE_NONE);
	} else {
		/* Timeout - RX should continue */
		inst_recv_finish(&srv->inst, OPCODE_ANY);
	}
	inst_error(&srv->inst, err, srv->ctx);
}

static void srv_send_handle(struct uart_dfu *dev)
{
	struct uart_dfu_srv *srv = dev->srv;
	inst_send_finish(&srv->inst, OPCODE_NONE);
}

static void srv_send_abort_handle(struct uart_dfu *dev, int err)
{
	struct uart_dfu_srv *srv = dev->srv;
	inst_send_finish(&srv->inst, OPCODE_NONE);
	inst_error(&srv->inst, err, srv->ctx);
}


/*
 * General helper functions 
 *****************************************************************************/

static bool pdu_validate(u8_t *buf, size_t len)
{
	struct uart_dfu_pdu *pdu;
	size_t arg_size;

	if (len < PDU_HEADER_SIZE) {
		return NULL;
	}

	pdu = (struct uart_dfu_pdu *) buf;
	arg_size = len - PDU_HEADER_SIZE;

	/* Validate message length. */
	switch (pdu->hdr.opcode) {
	case UART_DFU_OPCODE_INIT:
	case UART_DFU_OPCODE_WRITEH:
	case UART_DFU_OPCODE_OFFSET:
	case UART_DFU_OPCODE_DONE:
	{
		if (arg_size != PDU_ARG_SIZE) {
			/* All these PDUs have the same fixed length. */
			return false;
		}
		break;
	}
	case UART_DFU_OPCODE_WRITEC:
	{
		if (arg_size == 0) {
			/* Empty segments are disallowed. */
			return false;
		}
		break;
	}
	default:
	{
		/* Unknown opcode. */
		return false;
	}
	}

	return true;
}

static u32_t rx_size_get(struct uart_dfu *dev, enum inst_type *recv)
{
	u32_t size;
	u32_t srv_size;
	u32_t cli_size;
	struct cobs_decoder *decoder;

	*recv = TYPE_NONE;
	srv_size = srv_rx_size_get(dev->srv);
	if (srv_size > 0) {
		*recv |= TYPE_SRV;
	}
	cli_size = cli_rx_size_get(dev->cli);
	if (cli_size > 0) {
		*recv |= TYPE_CLI;
	}

	size = MAX(cli_size, srv_size);
	if (size > 0) {
		decoder = &dev->decoder;
		if (cobs_decode_in_frame(decoder)) {
			if (cobs_decode_current_size(decoder) < size) {
				size -= cobs_decode_current_size(decoder);
			} else {
				size = RX_WAIT_SIZE;
			}
		}	
	}
	return size;
}

static int send_ready(struct uart_dfu *dev, struct uart_dfu_inst *inst)
{
	int err;
	size_t tx_size;

	if (!atomic_cas(&dev->tx, 0, 1)) {
		/* TX already in progress. */
		return -EBUSY;
	}

	/* Choose buffer to transmit from.
	   The server has priority since it can not starve the client. */
	if (!inst_tx_rdy(inst)) {
		(void) atomic_set(&dev->tx, 0);
		return 0;
	}

	tx_size = inst->tx_size;
	err = inst_tx_start(inst);
	if (err != 0) {
		atomic_set(&dev->tx, 0);
		return -ECANCELED;
	} else {
		return (int) tx_size;
	}
}

static int recv_ready(struct uart_dfu *dev, enum inst_type *recv)
{
	int err;
	u32_t rx_size;

	*recv = TYPE_NONE;
	if (!atomic_cas(&dev->rx, 0, 1)) {
		return -EBUSY;
	}

	rx_size = rx_size_get(dev, recv);
	if (rx_size == 0) {
		return 0;
	}

	LOG_DBG("%s: enabling RX (size=%u)", dev->label, rx_size);
	err = uart_rx_enable(dev->dev, dev->rx_buf, rx_size, SYS_FOREVER_MS);
	if (err != 0) {
		LOG_ERR("%s: error enabling RX: %d.", dev->label, err);
		return -ECANCELED;	
	}
	return rx_size;
}

static enum inst_type timer_start_ready(struct uart_dfu *dev)
{
	struct uart_dfu_cli *cli = dev->cli;
	struct uart_dfu_srv *srv = dev->srv;
	enum inst_type started = TYPE_NONE;

	if (cli != NULL && inst_rx_timeout_start(&cli->inst)) {
		started |= TYPE_CLI;
		LOG_DBG("%s: started client RX timeout.", dev->label);
	}
	if (srv != NULL && inst_rx_timeout_start(&srv->inst)) {
		started |= TYPE_SRV;
		LOG_DBG("%s: started server RX timeout.", dev->label);
	}
	return started;
}

static void timer_stop(struct uart_dfu *dev, enum inst_type started)
{
	struct uart_dfu_cli *cli = dev->cli;
	struct uart_dfu_srv *srv = dev->srv;

	if (cli != NULL && started & TYPE_CLI) {
		inst_rx_timeout_stop(&cli->inst);
	}
	if (srv != NULL && started & TYPE_SRV) {
		inst_rx_timeout_stop(&srv->inst);
	}
}

static void recv_check(struct uart_dfu *dev)
{
	int err;
	enum inst_type recv;
	enum inst_type started;

	started = timer_start_ready(dev);
	err = recv_ready(dev, &recv);
	if (err < 0 && err != -EBUSY) {
		LOG_ERR("%s: error %d while starting RX.", dev->label, err);
		timer_stop(dev, started);
		if (recv & TYPE_CLI) {
			cli_recv_abort_handle(dev, -ECANCELED);
		}
		if (recv & TYPE_SRV) {
			srv_recv_abort_handle(dev, -ECANCELED);
		}
	}
}

static void send_check(struct uart_dfu *dev)
{
	int err;
	struct uart_dfu_cli *cli = dev->cli;
	struct uart_dfu_srv *srv = dev->srv;
	
	if (srv != NULL) {
		err = send_ready(dev, &srv->inst);
		if (err > 0 || err == -EBUSY) {
			return;
		}
		if (err < 0) {
			srv_recv_abort_handle(dev, -ECANCELED);
		}
	}
	if (cli != NULL) {
		err = send_ready(dev, &cli->inst);
		if (err > 0 || err == -EBUSY) {
			return;
		}
		if (err < 0) {
			cli_recv_abort_handle(dev, -ECANCELED);
		}
	}
}

/*
 * Workqueue handlers
 *****************************************************************************/

static void tx_dis_process(struct k_work *work)
{
	int err;
	struct tx_dis_info *info = CONTAINER_OF(work, struct tx_dis_info, work);
	struct uart_dfu *dev = CONTAINER_OF(info, struct uart_dfu, tx_dis);
	struct uart_dfu_cli *cli = dev->cli;
	struct uart_dfu_srv *srv = dev->srv;

	atomic_set(&dev->tx, 0);
	
	if (cli != NULL && info->buf == cli->buf) {
		err = inst_tx_abort_check(&cli->inst);
		if (err == 0) {
			cli_send_handle(dev);
		} else {
			cli_send_abort_handle(dev, err);
		}
	} else if (srv != NULL && info->buf == srv->buf) {
		err = inst_tx_abort_check(&srv->inst);
		if (err == 0) {
			srv_send_handle(dev);
		} else {
			srv_send_abort_handle(dev, err);
		}
	}

	/* Resume reception if applicable. */
	(void) recv_check(dev);
	/* Send queued transmissions */
	(void) send_check(dev);
}

static void rx_process(struct uart_dfu *dev,
		       const u8_t *buf,
		       size_t *offset,
		       size_t bound)
{
	int err;
	struct uart_dfu_pdu *pdu;
	size_t len;

	err = cobs_decode(&dev->decoder,
			  &len,
			  buf,
			  offset,
			  bound - *offset);
	if (err == -EMSGSIZE) {
		LOG_DBG("%s: PDU incomplete.", dev->label);
		return;
	}
	if (err == 0) {
		if (!pdu_validate(dev->rx_decoded_buf, len)) {
			LOG_ERR("%s: received invalid PDU.", dev->label);
		}
		pdu = (struct uart_dfu_pdu *) dev->rx_decoded_buf; 
		LOG_DBG("%s: received PDU.", dev->label);
		if (pdu->hdr.status) {
			cli_recv_handle(dev, pdu, len);
		} else {
			srv_recv_handle(dev, pdu, len);
		}
	} else {
		LOG_ERR("%s: error %d while decoding data.",
			dev->label,
			err);
	}
}

static void rx_rdy_process(struct k_work *work)
{
	struct rx_rdy_info *info = CONTAINER_OF(work, struct rx_rdy_info, work);
	struct uart_dfu *dev = CONTAINER_OF(info, struct uart_dfu, rx_rdy);
	size_t offset = info->offset;
	size_t bound = offset + info->len;
	const u8_t * buf = &info->buf[info->offset];

	LOG_DBG("%s: RX process (offset=%u, len=%u)",
		dev->label,
		info->offset,
		info->len);

	while (offset < bound) {
		rx_process(dev, buf, &offset, bound);	
	}
	
	/* Send queued transmissions */
	(void) send_check(dev);
}

static void rx_dis_process(struct k_work *work)
{
	int err;
	struct uart_dfu *dev = CONTAINER_OF(work, struct uart_dfu, rx_dis_work);
	struct uart_dfu_cli *cli = dev->cli;
	struct uart_dfu_srv *srv = dev->srv;

	atomic_set(&dev->rx, 0);

	/* Check for RX timeout/abort. */
	if (cli != NULL && (err = inst_rx_abort_check(&cli->inst)) != 0) {
		cli_recv_abort_handle(dev, err);
	}
	if (srv != NULL && (err = inst_rx_abort_check(&srv->inst)) != 0) {
		srv_recv_abort_handle(dev, err);
	}
	
	/* Resume reception */
	(void) recv_check(dev);
}


/*
 * Timer event handlers
 *****************************************************************************/

static void rx_timer_expired(struct k_timer *timer)
{
	struct uart_dfu *dev;
	struct uart_dfu_inst *inst;

	inst = CONTAINER_OF(timer, struct uart_dfu_inst, timer);
	dev = (struct uart_dfu *) inst->mod_ctx;
	if (dev == NULL) {
		return;
	}
	
	LOG_DBG("%s: RX timeout.", dev->label);
	inst_rx_timeout(inst);
}


/*
 * UART event handlers
 *****************************************************************************/

static void tx_done_handle(struct uart_dfu *dev,
			   struct uart_event_tx *evt,
			   bool aborted)
{
	struct uart_dfu_cli *cli = dev->cli;
	struct uart_dfu_srv *srv = dev->srv;

	if (cli != NULL && evt->buf == cli->buf) {
		inst_tx_finish(&cli->inst, aborted);
	} else if (srv != NULL && evt->buf == srv->buf) {
		inst_tx_finish(&srv->inst, aborted);
	}
	
	dev->tx_dis.buf = evt->buf;
	dev->tx_dis.len = evt->len;
	k_work_submit_to_queue(&dev->workqueue, &dev->tx_dis.work);	
}

static void uart_tx_done_handle(struct uart_dfu *dev, struct uart_event_tx *evt)
{
	LOG_DBG("%s: TX (len=%u)", dev->label, evt->len);	
	tx_done_handle(dev, evt, false);
}

static void uart_tx_aborted_handle(struct uart_dfu *dev,
				   struct uart_event_tx *evt)
{
	LOG_DBG("%s: TX aborted (len=%u)", dev->label, evt->len);
	tx_done_handle(dev, evt, true);
}

static void uart_rx_rdy_handle(struct uart_dfu *dev, struct uart_event_rx *evt)
{
	LOG_DBG("%s: RX (offset=%u, len=%u)",
		dev->label,
		evt->offset,
		evt->len);

	dev->rx_rdy.buf = evt->buf;
	dev->rx_rdy.offset = evt->offset;
	dev->rx_rdy.len = evt->len;

	k_work_submit_to_queue(&dev->workqueue, &dev->rx_rdy.work);
}

static void uart_rx_stopped_handle(struct uart_dfu *dev,
				   struct uart_event_rx_stop *evt)
{
	struct uart_dfu_cli *cli = dev->cli;
	struct uart_dfu_srv *srv = dev->srv;

	LOG_ERR("%s: RX stopped (reason=%d)", dev->label, evt->reason);

#if 0
	/* TODO: decide on how to handle this. The STOPPED event occurs during
	         flashing, so aborting makes it impossible to debug */
	/* Abort any ongoing reception. */
	if (cli != NULL && inst_rx_rdy(&cli->inst)) {
		inst_rx_set_abort(&cli->inst, true);
	}
	if (srv != NULL && inst_rx_rdy(&srv->inst)) {
		inst_rx_set_abort(&srv->inst, true);
	}
#endif
}

static void uart_rx_disabled_handle(struct uart_dfu *dev)
{
	k_work_submit_to_queue(&dev->workqueue, &dev->rx_dis_work);
}

static void uart_async_cb(struct uart_event *evt, void *user_data)
{
	struct uart_dfu *dev = (struct uart_dfu *) user_data;

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG("%s: UART_TX_DONE", dev->label);
		uart_tx_done_handle(dev, &evt->data.tx);
		break;
	case UART_TX_ABORTED:
		LOG_DBG("%s: UART_TX_ABORTED", dev->label);
		uart_tx_aborted_handle(dev, &evt->data.tx);
		break;
	case UART_RX_RDY:
		LOG_DBG("%s: UART_RX_RDY", dev->label);
		uart_rx_rdy_handle(dev, &evt->data.rx);
		break;
	case UART_RX_DISABLED:
		LOG_DBG("%s: UART_RX_DISABLED", dev->label);
		uart_rx_disabled_handle(dev);
		break;
	case UART_RX_STOPPED:
		LOG_DBG("%s: UART_RX_STOPPED", dev->label);
		uart_rx_stopped_handle(dev, &evt->data.rx_stop);
		break;
	case UART_RX_BUF_REQUEST:
	case UART_RX_BUF_RELEASED:
	default:
		break;
	}
}


/*****************************************************************************
* API functions
*****************************************************************************/

int uart_dfu_init(size_t idx)
{
	int err;
	struct uart_dfu *dev;
	struct uart_config cfg;

	dev = dfu_device_get(idx);
	if (dev == NULL) {
		return -EINVAL;
	}

	printk("Initializing UART DFU on %s.\n", dev->label);

	if (dev->dev != NULL) {
		printk("Error: multiple initialization on %s.\n", dev->label);
		return -EBUSY;
	}

	/* Driver init */
	dev->dev = device_get_binding(dev->label);
	if (dev->dev == NULL) {
		return -EACCES;
	}

	err = uart_config_get(dev->dev, &cfg);
	if (err != 0) {
		dev->dev = NULL;
		return -EIO;
	}
	dev->baudrate = cfg.baudrate;

	err = uart_callback_set(dev->dev, uart_async_cb, dev);
	if (err != 0) {
		dev->dev = NULL;
		return -EIO;
	}

	/* Module init */
	(void) atomic_set(&dev->rx, 0);
	(void) atomic_set(&dev->tx, 0);
	(void) cobs_decoder_init(&dev->decoder, dev->rx_decoded_buf);
	k_work_init(&dev->rx_rdy.work, rx_rdy_process);
	k_work_init(&dev->rx_dis_work, rx_dis_process);
	k_work_init(&dev->tx_dis.work, tx_dis_process);
	dev->cli = NULL;
	dev->srv = NULL;
	k_work_q_start(&dev->workqueue,
		       dev->stack_area,
		       UART_DFU_STACK_SIZE,
		       CONFIG_UART_DFU_LIBRARY_THREAD_PRIO);

	printk("Initialized UART DFU on %s.\n", dev->label);

	return 0;
}

static int uart_dfu_sys_init(struct device *dev)
{
	ARG_UNUSED(dev);

	int err;

	for (size_t idx = 0; idx < ARRAY_SIZE(devices); idx++) {
		if (devices[idx] != NULL) {
			err = uart_dfu_init(idx);
			if (err != 0) {
				return err;
			}
		}
	}

	return 0;
}

int uart_dfu_uninit(size_t idx)
{
	return -ENOSYS;
}

int uart_dfu_cli_init(struct uart_dfu_cli *client,
		     struct uart_dfu_cli_cb *callbacks,
		     uart_dfu_error_t error_cb,
		     void *context)
{
	if (client == NULL			||
	    callbacks == NULL			||
	    callbacks->status_cb == NULL	||
	    callbacks->offset_cb == NULL	||
	    error_cb == NULL) {
		return -EINVAL;
	}

	inst_init(&client->inst,
		  client->buf,
		  UART_DFU_CLI_TX_BUF_SIZE,
		  error_cb);
	memset(&client->fragment, 0, sizeof(client->fragment));
	client->cb = *callbacks;
	client->ctx = context;
	return 0;
}

int uart_dfu_cli_bind(size_t idx, struct uart_dfu_cli *client)
{
	struct uart_dfu *dev;
	struct uart_dfu_cli *cli;

	if (client == NULL) {
		return -EINVAL;
	}

	dev = dfu_device_get(idx);
	if (dev == NULL) {
		return -EINVAL;
	}

	cli = dev->cli;
	if (cli != NULL) {
		return -EBUSY;
	}

	dev->cli = client;
	client->inst.mod_ctx = dev;
	return 0;
}

int uart_dfu_cli_unbind(size_t idx)
{
	struct uart_dfu *dev;
	struct uart_dfu_cli *cli;

	dev = dfu_device_get(idx);
	if (dev == NULL) {
		return -EINVAL;
	}

	cli = dev->cli;
	if (cli == NULL) {
		return 0;
	}
	if (inst_send_active(&cli->inst) || inst_recv_active(&cli->inst)) {
		return -EBUSY;
	}

	cli->inst.mod_ctx = dev;
	dev->cli = NULL;
	return 0;
}

static int api_cli_get(size_t idx, struct uart_dfu_cli **cli)
{
	struct uart_dfu *dev = dfu_device_get(idx);
	if (dev == NULL) {
		return -EINVAL;
	}
	if (dev->dev == NULL || dev->cli == NULL) {
		return -EACCES;
	}
	*cli = dev->cli;
	return 0;
}

static int api_cli_send(struct uart_dfu_cli *cli, struct uart_dfu_pdu *pdu)
{
	int err;
	struct uart_dfu *dev = (struct uart_dfu *) cli->inst.mod_ctx;
	enum inst_type recv;

	err = inst_send_cond(&cli->inst,
			     OPCODE_NONE,
			     &pdu->hdr,
			     &pdu->args,
			     sizeof(pdu->args));
	if (err != 0) {
		return err;
	}
	err = inst_recv_cond(&cli->inst, OPCODE_NONE, pdu->hdr.opcode);
	if (err != 0) {
		inst_send_cancel(&cli->inst);
		return -ECANCELED;
	}

	err = send_ready(dev, &cli->inst);
	if (err <= 0 && err != -EBUSY) {
		goto cleanup;
	}
	err = recv_ready(dev, &recv);
	if (err > 0 || err == -EBUSY) {
		LOG_DBG("api cli send success");
		return 0;
	}

cleanup:
	inst_recv_cancel(&cli->inst);
	inst_send_cancel(&cli->inst);
	return -ECANCELED;
}

int uart_dfu_cli_init_send(size_t idx, size_t file_size)
{
	int err;
	struct uart_dfu_cli *cli;
	struct uart_dfu_pdu pdu;

	LOG_DBG("cli init send.");

	if (file_size > INT32_MAX) {
		return -ENOMEM;
	}
	
	err = api_cli_get(idx, &cli);
	if (err != 0) {
		return err;
	}

	memset(&pdu, 0, sizeof(pdu));
	pdu.hdr.opcode = UART_DFU_OPCODE_INIT;
	pdu.args.init.file_size = (u32_t) file_size;

	return api_cli_send(cli, &pdu);
}

int uart_dfu_cli_write_send(size_t idx,
			    const u8_t *const fragment_buf,
			    size_t fragment_size)
{
	int err;
	struct uart_dfu_cli *cli;
	struct uart_dfu_pdu pdu;

	LOG_DBG("cli write send.");
	
	if (fragment_buf == NULL || fragment_size == 0) {
		return -EINVAL;
	}

	if (fragment_size > INT32_MAX) {
		return -ENOMEM;
	}

	err = api_cli_get(idx, &cli);
	if (err != 0) {
		return err;
	}

	if (inst_send_active(&cli->inst)) {
		return -ECANCELED;
	}

	buf_in_cfg(&cli->fragment, fragment_buf, 0, fragment_size);
	memset(&pdu, 0, sizeof(pdu));
	pdu.hdr.opcode = UART_DFU_OPCODE_WRITEH;
	pdu.args.writeh.fragment_size = fragment_size;
	err = api_cli_send(cli, &pdu);
	if (err != 0) {
		buf_in_cfg(&cli->fragment, NULL, 0, 0);
	}
	return err;
}

int uart_dfu_cli_offset_send(size_t idx)
{
	int err;
	struct uart_dfu_cli *cli;
	struct uart_dfu_pdu pdu; 

	LOG_DBG("cli offset send.");
	
	err = api_cli_get(idx, &cli);
	if (err != 0) {
		return err;
	}

	memset(&pdu, 0, sizeof(pdu));
	pdu.hdr.opcode = UART_DFU_OPCODE_OFFSET;
	return api_cli_send(cli, &pdu);
}

int uart_dfu_cli_done_send(size_t idx, bool successful)
{
	int err;
	struct uart_dfu_cli *cli;
	struct uart_dfu_pdu pdu;

	LOG_DBG("cli done send.");
	
	err = api_cli_get(idx, &cli);
	if (err != 0) {
		return err;
	}

	memset(&pdu, 0, sizeof(pdu));
	pdu.hdr.opcode = UART_DFU_OPCODE_DONE;
	pdu.args.done.success = successful;
	return api_cli_send(cli, &pdu);
}

int uart_dfu_cli_stop(size_t idx)
{
	int err;
	struct uart_dfu_cli *cli;

	err = api_cli_get(idx, &cli);
	if (err != 0) {
		return err;
	}
	return inst_active_abort(&cli->inst);
}

int uart_dfu_srv_init(struct uart_dfu_srv *server,
		      u8_t *fragment_buf,
		      size_t fragment_max_size,
		      struct uart_dfu_srv_cb *callbacks,
		      uart_dfu_error_t error_cb,
		      void *context)
{
	if (server == NULL			||
	    fragment_buf == NULL		||
	    fragment_max_size == 0		||
	    callbacks == NULL			||
	    callbacks->init_cb == NULL		||
	    callbacks->write_cb == NULL		||
	    callbacks->offset_cb == NULL	||
	    callbacks->done_cb == NULL		||
	    error_cb == NULL) {
		return -EINVAL;
	}

	inst_init(&server->inst,
		  server->buf,
		  UART_DFU_SRV_TX_BUF_SIZE,
		  error_cb);
	memset(&server->fragment, 0, sizeof(server->fragment));
	server->fragment.buf = fragment_buf;
	server->fragment.max_size = fragment_max_size;	
	server->cb = *callbacks;
	server->ctx = context;
	return 0;
}

static int api_srv_get(size_t idx, struct uart_dfu_srv **srv)
{
	struct uart_dfu *dev = dfu_device_get(idx);
	if (dev == NULL) {
		return -EINVAL;
	}
	if (dev->dev == NULL || dev->srv == NULL) {
		return -EACCES;
	}
	*srv = dev->srv;
	return 0;
}

int uart_dfu_srv_bind(size_t idx, struct uart_dfu_srv *server)
{
	struct uart_dfu *dev;
	struct uart_dfu_srv *srv;

	if (server == NULL) {
		return -EINVAL;
	}

	dev = dfu_device_get(idx);
	if (dev == NULL) {
		return -EINVAL;
	}

	srv = dev->srv;
	if (srv != NULL) {
		return -EBUSY;
	}

	dev->srv = server;
	server->inst.mod_ctx = dev;
	return 0;
}

int uart_dfu_srv_unbind(size_t idx)
{
	struct uart_dfu *dev;
	struct uart_dfu_srv *srv;

	dev = dfu_device_get(idx);
	if (dev == NULL) {
		return -EINVAL;
	}

	srv = dev->srv;
	if (srv == NULL) {
		return 0;
	}
	
	if (inst_send_active(&srv->inst) || inst_recv_active(&srv->inst)) {
		return -EBUSY;
	}

	srv->inst.mod_ctx = NULL;
	dev->srv = NULL;
	return 0;
}

int uart_dfu_srv_enable(size_t idx)
{
	int err;
	struct uart_dfu *dev;
	struct uart_dfu_srv *srv;
	enum inst_type recv;

	err = api_srv_get(idx, &srv);
	if (err != 0) {
		return err;
	}

	err = inst_recv_cond(&srv->inst, OPCODE_NONE, OPCODE_ANY);
	if (err != 0) {
		return err;
	}

	dev = (struct uart_dfu *) srv->inst.mod_ctx;
	err = recv_ready(dev, &recv);

	if (err > 0 || err == -EBUSY) {
		return 0;
	} else {
		(void) inst_recv_cancel(&srv->inst);
		return -ECANCELED;
	}
}

int uart_dfu_srv_disable(size_t idx)
{
	int err;
	struct uart_dfu_srv *srv;

	err = api_srv_get(idx, &srv);
	if (err != 0) {
		return err;
	}
	return inst_active_abort(&srv->inst);
}


/*****************************************************************************
* System initialization hooks
*****************************************************************************/

#ifdef CONFIG_UART_DFU_SYS_INIT
SYS_INIT(uart_dfu_sys_init, APPLICATION, CONFIG_UART_DFU_INIT_PRIORITY);
#endif
