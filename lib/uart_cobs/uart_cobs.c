/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>

#define UART_COBS_DT DT_CHOSEN(nordic_cobs_uart_controller)

#if !DT_NODE_EXISTS(UART_COBS_DT)
#error "Missing /chosen devicetree node: nordic,cobs-uart-controller"

#elif !DT_NODE_HAS_STATUS(UART_COBS_DT, okay)
#error "nordic,cobs-uart-controller not enabled"

#elif !DT_PROP(UART_COBS_DT, hw_flow_control)
#error "Hardware flow control not enabled for nordic,cobs-uart-controller"

#else

#include <sys/time_units.h>
#include <sys/atomic.h>
#include <logging/log.h>
#include <drivers/uart.h>

#include <uart_cobs.h>
#include "cobs.h"

/* Compile-time check that max payload size is set correctly. */
BUILD_ASSERT(UART_COBS_MAX_PAYLOAD_LEN == COBS_MAX_DATA_BYTES,
	     "Max payload size should be equal to max COBS size.");

#define LOG_DBG_DEV(...) LOG_DBG(DT_LABEL(UART_COBS_DT) ": " __VA_ARGS__)

LOG_MODULE_REGISTER(uart_cobs, CONFIG_UART_COBS_LOG_LEVEL);


static void sw_evt_handle(const struct uart_cobs_evt *const evt, void *ctx);
static void no_evt_handle(const struct uart_cobs_evt *const evt, void *ctx);


enum op_status {
	STATUS_OFF	= 0,
	STATUS_ON	= 1
};

struct {
	struct device *dev;
	struct {
		atomic_t status;
		struct cobs_enc_buf buf;
		size_t len;
	} tx;
	struct {
		atomic_t status;
		bool processing;
		struct k_timer timer;
		struct cobs_dec decoder;
		uint8_t decoded_buf[COBS_MAX_DATA_BYTES];
		struct {
			uint8_t buf[COBS_MAX_BYTES];
			size_t offset;
			size_t len;
		} raw;
		size_t expected_len;
	} rx;
	struct {
		atomic_ptr_t current;
		struct uart_cobs_user *idle;
		struct {
			struct uart_cobs_user *from;
			struct uart_cobs_user *to;
			int err;
		} pending;
	} user;
	struct {
		struct k_work rx_dis;
		struct k_work tx_dis;
		struct k_work_q q;
	} work;
} state;

K_THREAD_STACK_DEFINE(work_q_stack_area,
		      CONFIG_UART_COBS_THREAD_STACK_SIZE);

UART_COBS_USER_DEFINE(sw_user, sw_evt_handle, NULL);
UART_COBS_USER_DEFINE(no_user, no_evt_handle, NULL);


static bool status_error_set(atomic_t *status, int err)
{
	return atomic_cas(status, STATUS_ON, err);
}

static inline bool rx_error_set(int err)
{
	return status_error_set(&state.rx.status, err);
}

static inline bool tx_error_set(int err)
{
	return status_error_set(&state.tx.status, err);
}

static inline void evt_send(const struct uart_cobs_evt *const evt)
{
	struct uart_cobs_user *user;
	user = (struct uart_cobs_user *) atomic_ptr_get(&state.user.current);
	user->cb(evt, user->ctx);
}

static void send_evt_end(enum uart_cobs_evt_type type, int err)
{
	struct uart_cobs_evt evt;
	evt.type = type;
	evt.data.err = err;
	evt_send(&evt);
}

static void user_start(void)
{
	struct uart_cobs_evt evt;
	evt.type = UART_COBS_EVT_USER_START;
	evt_send(&evt);
}

static void user_end(struct uart_cobs_user *from, int err)
{
	if (from != NULL) {
		struct uart_cobs_evt evt;
		evt.type = UART_COBS_EVT_USER_END;
		evt.data.err = err;
		from->cb(&evt, from->ctx);
	}
}

static bool user_sw_ready(void)
{
	bool ready = true;

	/* Check current RX status. */
	switch (atomic_get(&state.rx.status)) {
	case STATUS_OFF:
		state.rx.processing = false;
		uart_cobs_rx_timeout_stop(&state.user.current);
		break;
	case STATUS_ON:
		(void) uart_cobs_rx_stop(&state.user.current);
		ready = false;
		break;
	default:
		/* Error state. */
		ready = false;
		break;
	}

	/* Check current TX status. */
	switch (atomic_get(&state.tx.status)) {
	case STATUS_OFF:
		break;
	case STATUS_ON:
		(void) uart_cobs_tx_stop(&state.user.current);
		ready = false;
		break;
	default:
		/* Error state. */
		ready = false;
		break;
	}

	return ready;
}

/* TODO: set TX len to 0 */
static void user_sw_finish(void)
{
	LOG_DBG_DEV("User end: %lu",
		    (long unsigned int) state.user.pending.from);
	user_end(state.user.pending.from, state.user.pending.err);

	LOG_DBG_DEV("User start: %lu",
		    (long unsigned int) state.user.pending.to);
	__ASSERT(atomic_get(&state.rx.status) == STATUS_OFF &&
		 atomic_get(&state.tx.status) == STATUS_OFF,
		 "Expected RX and TX to be off before switching user");
	(void) atomic_ptr_set(&state.user.current, state.user.pending.to);
	user_start();
}

static int user_sw_prepare(struct uart_cobs_user *from,
			   struct uart_cobs_user *to, int err)
{
	/* Switch between sessions by passing through a temporary "SW" session
	 * where RX/TX can be stopped and residual events can be safely caught
	 * without interfering with normal sessions.
	 */

	if (!atomic_ptr_cas(&state.user.current, from, &sw_user)) {
		return -EBUSY;
	}

	LOG_DBG_DEV("User switch from %lu to %lu",
		    (long unsigned int) from, (long unsigned int) to);

	state.user.pending.from	= from;
	state.user.pending.to	= to;
	state.user.pending.err	= err;

	if (user_sw_ready()) {
		/* Finish synchronously. */
		user_sw_finish();
		return 0;
	} else {
		/* Finish asynchronously (via sw_evt_handle()). */
		/* FIXME: is it necessary to return an error code? */
		return -EINPROGRESS;
	}
}

static void no_evt_handle(const struct uart_cobs_evt *const evt)
{
	ARG_UNUSED(evt);

	/* Do nothing. */
}

static void sw_evt_handle(const struct uart_cobs_evt *const evt)
{
	ARG_UNUSED(evt);

	if (user_sw_ready()) {
		user_sw_finish();
	}
}

static int rx_start(size_t len)
{
	int err = uart_rx_enable(state.dev, state.rx.raw.buf,
				 len, SYS_FOREVER_MS);
	if (!err) {
		LOG_DBG_DEV("RX started");
	} else {
		(void) atomic_set(&state.rx.status, STATUS_OFF);
		LOG_DBG_DEV("RX failed to start (drv err %d)", err);
	}

	return err;
}

static void rx_resume(void)
{
	size_t len = state.rx.expected_len;

	if (cobs_dec_in_frame(&state.rx.decoder)) {
		if (cobs_dec_current_len(&state.rx.decoder) < len) {
			len -= cobs_dec_current_len(&state.rx.decoder);
		} else {
			len = 1;
		}
	}
	state.rx.expected_len = len;
	(void) rx_start(len);
}

static void rx_process(void)
{
	int ret = 0;
	const uint8_t *buf = &state.rx.raw.buf[state.rx.raw.offset];
	struct uart_cobs_evt evt;

	evt.type = UART_COBS_EVT_RX;
	evt.data.rx.buf = state.rx.decoded_buf;

	for (size_t i = 0; i < state.rx.raw.len; i++) {
		ret = cobs_decode_step(&state.rx.decoder, buf[i]);
		if (ret == -EINPROGRESS) {
			continue;
		} else if (ret < 0) {
			LOG_DBG_DEV("COBS decode error (%d)", ret);
			break;
		}
		LOG_DBG_DEV("RX PDU event");
		evt.data.rx.len = ret;
		evt_send(&evt);

		if (!state.rx.processing) {
			/* Processing was aborted due to session switch. */
			ret = -ECANCELED;
			break;
		} else if (atomic_get(&state.rx.status) == STATUS_ON) {
			/* uart_cobs_rx_start() was called in event handler. */
			ret = -EAGAIN;
		}
	}

	state.rx.processing = false;
	if (ret < 0 && ret != -EINPROGRESS && ret != -EAGAIN) {
		LOG_DBG_DEV("RX processing aborted");
	} else {
		LOG_DBG_DEV("RX processing finished");
		if (ret == -EINPROGRESS) {
			LOG_DBG_DEV("RX PDU incomplete");
		}
	}
	if (ret < 0) {
		rx_resume();
	}
}

static void rx_dis_process(struct k_work *work)
{
	ARG_UNUSED(work);

	atomic_val_t status = atomic_set(&state.rx.status, STATUS_OFF);

	if (status == STATUS_ON) {
		if (state.rx.processing) {
			rx_process();
		}
	} else if (status == -EAGAIN) {
		/* Malformed data. Reset and resume reception. */
		(void) atomic_set(&state.rx.status, STATUS_ON);
		state.rx.processing = false;
		cobs_dec_reset(&state.rx.decoder);
		rx_resume();
	} else {
		/* Fatal error. */
		LOG_DBG_DEV("RX stopped with error: %d", status);
		state.rx.processing = false;
		send_evt_end(UART_COBS_EVT_RX_END, status);
	}
}

static void tx_dis_process(struct k_work *work)
{
	ARG_UNUSED(work);

	atomic_val_t status = atomic_set(&state.tx.status, STATUS_OFF);

	if (status < 0) {
		LOG_DBG_DEV("TX stopped with error %d", status);
		send_evt_end(UART_COBS_EVT_TX_END, status);
	} else {
		LOG_DBG_DEV("TX complete event");
		send_evt_end(UART_COBS_EVT_TX_END, 0);
	}
}

static void rx_timer_expired(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	if (rx_error_set(-ETIMEDOUT)) {
		(void) uart_rx_disable(state.dev);
	}
}

static void uart_tx_handle(struct uart_event_tx *evt, bool aborted)
{
	LOG_DBG_DEV("TX %s (len=%u)", (aborted ? "aborted" : ""), evt->len);

	if (aborted) {
		/* This will not overwrite the user abort error if it is set. */
		(void) tx_error_set(-ETIMEDOUT);
	}
	k_work_submit_to_queue(&state.work.q, &state.work.tx_dis);
}

static void uart_rx_rdy_handle(struct uart_event_rx *evt)
{
	__ASSERT(evt->buf == state.rx.raw.buf,
		 "Expected event buf and local buf to be the same");
	__ASSERT(!state.rx.processing,
		 "RX should not be running while processing");

	LOG_DBG_DEV("RX (offset=%u, len=%u)", evt->offset, evt->len);

	state.rx.raw.offset = evt->offset;
	state.rx.raw.len = evt->len;
	state.rx.processing = true;
}

static void uart_rx_stopped_handle(struct uart_event_rx_stop *evt)
{
	LOG_DBG_DEV("RX stopped (reason=%d)", evt->reason);

	switch (evt->reason) {
	case UART_ERROR_OVERRUN:
	case UART_ERROR_PARITY:
	case UART_ERROR_FRAMING:
		(void) rx_error_set(-EAGAIN);
		break;
	case UART_BREAK:
	default:
		(void) rx_error_set(-ENETDOWN);
		break;
	}
}

static void uart_rx_disabled_handle(void)
{
	k_work_submit_to_queue(&state.work.q, &state.work.rx_dis);
}

static void uart_async_cb(struct device *dev,
			  struct uart_event *evt,
			  void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	switch (evt->type) {
	case UART_TX_DONE:
		LOG_DBG_DEV("UART_TX_DONE");
		uart_tx_handle(&evt->data.tx, false);
		break;
	case UART_TX_ABORTED:
		LOG_DBG_DEV("UART_TX_ABORTED");
		uart_tx_handle(&evt->data.tx, true);
		break;
	case UART_RX_RDY:
		LOG_DBG_DEV("UART_RX_RDY");
		uart_rx_rdy_handle(&evt->data.rx);
		break;
	case UART_RX_DISABLED:
		LOG_DBG_DEV("UART_RX_DISABLED");
		uart_rx_disabled_handle();
		break;
	case UART_RX_STOPPED:
		LOG_DBG_DEV("UART_RX_STOPPED");
		uart_rx_stopped_handle(&evt->data.rx_stop);
		break;
	case UART_RX_BUF_REQUEST:
	case UART_RX_BUF_RELEASED:
	default:
		break;
	}
}

static int uart_cobs_sys_init(struct device *dev)
{
	ARG_UNUSED(dev);

	/* Driver init. */
	state.dev = device_get_binding(DT_LABEL(UART_COBS_DT));
	if (state.dev == NULL) {
		return -EACCES;
	}

	int err = uart_callback_set(state.dev, uart_async_cb, NULL);
	if (err) {
		state.dev = NULL;
		return -EIO;
	}

	/* Module init. */
	(void) atomic_set(&state.rx.status, STATUS_OFF);
	(void) atomic_set(&state.tx.status, STATUS_OFF);

	k_timer_init(&state.rx.timer, rx_timer_expired, NULL);
	k_timer_user_data_set(&state.rx.timer, NULL);

	cobs_dec_init(&state.rx.decoder, state.rx.decoded_buf);

	state.user.idle = &no_user;
	(void) atomic_ptr_set(&state.user.current, state.user.idle);

	k_work_init(&state.work.rx_dis, rx_dis_process);
	k_work_init(&state.work.tx_dis, tx_dis_process);

	k_work_q_start(&state.work.q, work_q_stack_area,
		       K_THREAD_STACK_SIZEOF(work_q_stack_area),
		       CONFIG_UART_COBS_THREAD_PRIO);
	return 0;
}

int uart_cobs_init(void)
{
	if (state.dev == NULL) {
		return uart_cobs_sys_init(NULL);
	} else {
		/* Already initialized. */
		return 0;
	}
}

int uart_cobs_user_start(struct uart_cobs_user *user)
{
	if (user == NULL) {
		return -EINVAL;
	}
	if (atomic_ptr_get(&state.user.current) == user) {
		return -EALREADY;
	} else {
		return user_sw_prepare(state.user.idle, user, 0);
	}
}

int uart_cobs_user_end(struct uart_cobs_user *user, int err)
{
	if (user == NULL) {
		return -EINVAL;
	}
	return user_sw_prepare(user, state.user.idle, err);
}

bool uart_cobs_user_active(struct uart_cobs_user *user)
{
	return atomic_get(&state.user.current) == user;
}

int uart_cobs_default_user_set(struct uart_cobs_user *user)
{
	if (user == NULL) {
		/* Switch to the unset idle state if NULL is passed */
		user = &no_user;
	}

	/* Switch to empty idle handler first to lock the user state,
	   preventing other switches from messing up the state. */
	if (!atomic_ptr_cas(&state.user.current, state.user.idle, &no_user)) {
		return -EBUSY;
	}
	
	struct uart_cobs_user *prev_user = state.user.idle;
	state.user.idle = user;
	if (prev_user == user) {
		return -EALREADY;
	}

	(void) user_sw_prepare(prev_user, user, 0);
	return 0;
}

int uart_cobs_tx_buf_write(struct uart_cobs_user *user,
			   uint8_t *data, size_t len)
{
	if (data == NULL) {
		return -EINVAL;
	}
	if (atomic_get(&state.user.current) != user) {
		return -EACCES;
	}

	size_t new_len = state.tx.len + len;
	if (new_len > COBS_MAX_DATA_BYTES) {
		return -ENOMEM;
	}

	if (atomic_get(&state.tx.status) != STATUS_OFF) {
		return -EBUSY;
	}

	memcpy(&state.tx.buf.buf[state.tx.len], data, len);
	state.tx.len = new_len;
	return 0;
}

int uart_cobs_tx_buf_clear(struct uart_cobs_user *user)
{
	if (atomic_get(&state.user.current) != user) {
		return -EACCES;
	}
	if (atomic_get(&state.tx.status) != STATUS_OFF) {
		return -EBUSY;
	}

	state.tx.len = 0;
	return 0;
}

int uart_cobs_tx_start(struct uart_cobs_user *user, int timeout)
{
	if (atomic_get(&state.user.current) != user) {
		return -EACCES;
	}
	if (!atomic_cas(&state.tx.status, STATUS_OFF, STATUS_ON)) {
		LOG_DBG_DEV("TX was already started");
		return -EBUSY;
	}

	int err = cobs_encode(&state.tx.buf, state.tx.len);
	if (err) {
		(void) atomic_set(&state.tx.status, STATUS_OFF);
		return err;
	}

	err = uart_tx(state.dev, (uint8_t *) &state.tx.buf,
		      COBS_ENCODED_SIZE(state.tx.len), timeout);
	if (err) {
		LOG_DBG_DEV("TX failed to start (drv err %d)", err);
		(void) atomic_set(&state.tx.status, STATUS_OFF);
		return err;
	}
	return 0;
}

int uart_cobs_tx_stop(struct uart_cobs_user *user)
{
	if (atomic_get(&state.user.current) != user) {
		return -EACCES;
	}
	if (tx_error_set(-ECONNABORTED)) {
		return uart_tx_abort(state.dev);
	} else {
		/* Already aborted or aborting. */
		return 0;
	}
}

int uart_cobs_rx_start(struct uart_cobs_user *user, size_t len)
{
	if (atomic_get(&state.user.current) != user) {
		return -EACCES;
	}
	if (len == 0 || len > COBS_MAX_DATA_BYTES) {
		return -EINVAL;
	}
	if (!atomic_cas(&state.rx.status, STATUS_OFF, STATUS_ON)) {
		LOG_DBG_DEV("RX was already started");
		return -EBUSY;
	}

	/* Generally, do not start RX during buffer processing. */
	if (!state.rx.processing) {
		int err = rx_start(COBS_ENCODED_SIZE(len));
		if (err) {
			return err;
		}
	} else if (uart_cobs_in_work_q_thread()) {
		/* RX will start after buffer processing finishes. */
	} else {
		/* TODO: resolve issue where higher prio thread enters here. */
		LOG_DBG_DEV("RX failed to start (processing)");
		(void) atomic_set(&state.rx.status, STATUS_OFF);
		return -EBUSY;
	}

	state.rx.expected_len = COBS_ENCODED_SIZE(len);
	return 0;
}

int uart_cobs_rx_stop(struct uart_cobs_user *user)
{
	if (atomic_get(&state.user.current) != user) {
		return -EACCES;
	}
	if (rx_error_set(-ECONNABORTED)) {
		return uart_rx_disable(state.dev);
	} else {
		/* Already aborted or aborting. */
		return 0;
	}
}

int uart_cobs_rx_timeout_start(struct uart_cobs_user *user, int timeout)
{
	if (atomic_get(&state.user.current) != user) {
		return -EACCES;
	}
	k_timer_start(&state.rx.timer, K_MSEC(timeout), K_NO_WAIT);
	return 0;
}

int uart_cobs_rx_timeout_stop(struct uart_cobs_user *user)
{
	if (atomic_get(&state.user.current) != user) {
		return -EACCES;
	}
	k_timer_stop(&state.rx.timer);
	return 0;
}

bool uart_cobs_in_work_q_thread(void)
{
	return k_current_get() == &state.work.q.thread;
}

SYS_INIT(uart_cobs_sys_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);


#endif /* DT_NODE_EXISTS(UART_COBS_DT) */
