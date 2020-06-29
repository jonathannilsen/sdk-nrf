/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <sys/time_units.h>
#include <sys/atomic.h>
#include <logging/log.h>
#include <drivers/uart.h>

#include <uart_dfu.h>
#include <uart_dfu_types.h>
#include <uart_dfu_srv.h>
#include <uart_dfu_cli.h>
#include <cobs.h>

/*****************************************************************************
* Macros
*****************************************************************************/

#define EVT_SEND	 (evt_cb[atomic_get(&state.sess.current)])

#define LOG_DBG_DEV(...) LOG_DBG(DT_LABEL(UART_DFU_DT) ": " __VA_ARGS__)

#define UART_DFU_DT	 DT_CHOSEN(nordic_dfu_uart_controller)
#define MIN_TIMEOUT	 ceiling_fraction(COBS_MAX_BYTES * 8 * MSEC_PER_SEC, \
					  DT_PROP(UART_DFU_DT, current_speed))
#define TIMEOUT_VALIDATE(_timeout) \
	BUILD_ASSERT(MIN_TIMEOUT < _timeout, \
		     #_timeout " is too low for " DT_LABEL(UART_DFU_DT))

/* Compile-time validation of chosen UART controller. */
BUILD_ASSERT(DT_NODE_EXISTS(UART_DFU_DT),
	     "Missing /chosen devicetree node: nordic,dfu-uart-controller");
BUILD_ASSERT(DT_NODE_HAS_STATUS(UART_DFU_DT, okay),
	     DT_LABEL(UART_DFU_DT) " not enabled");
BUILD_ASSERT(DT_PROP(UART_DFU_DT, hw_flow_control),
	     "Hardware flow control not enabled for " DT_LABEL(UART_DFU_DT));

#if CONFIG_UART_DFU_CLI
TIMEOUT_VALIDATE(CONFIG_UART_DFU_CLI_RESPONSE_TIMEOUT);
TIMEOUT_VALIDATE(CONFIG_UART_DFU_CLI_SEND_TIMEOUT);
#endif
#if CONFIG_UART_DFU_SRV
TIMEOUT_VALIDATE(CONFIG_UART_DFU_SRV_RESPONSE_TIMEOUT);
TIMEOUT_VALIDATE(CONFIG_UART_DFU_SRV_SEND_TIMEOUT);
#endif

LOG_MODULE_REGISTER(uart_dfu, CONFIG_UART_DFU_LIBRARY_LOG_LEVEL);

/*****************************************************************************
* Weak function declarations
*****************************************************************************/

void __weak uart_dfu_srv_idle_evt_handle(const struct uart_dfu_evt *const evt)
{
}

void __weak uart_dfu_srv_evt_handle(const struct uart_dfu_evt *const evt)
{
}

void __weak uart_dfu_cli_evt_handle(const struct uart_dfu_evt *const evt)
{
}

/*****************************************************************************
* Forward declarations
*****************************************************************************/

static void uart_dfu_sw_evt_handle(const struct uart_dfu_evt *const evt);

/*****************************************************************************
* Static variables
*****************************************************************************/

enum state_flags {
	FLAG_ON,
	FLAG_TIMEOUT,
	FLAG_USER_ABORT,
	FLAG_PROCESSING,
	FLAG_COUNT
};

struct {
	struct device *dev;
	struct {
		ATOMIC_DEFINE(flags, FLAG_COUNT);
		struct cobs_enc_buf buf;
	} tx;
	struct {
		ATOMIC_DEFINE(flags, FLAG_COUNT);
		atomic_t stop_reason;
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
		atomic_t current;
		struct {
			atomic_t flag;
			enum uart_dfu_sess from;
			enum uart_dfu_sess to;
			int err;
		} pending;
	} sess;
	struct {
		struct k_work rx_dis;
		struct k_work tx_dis;
		struct k_work_q q;
	} work;
} state;

K_THREAD_STACK_DEFINE(work_q_stack_area,
		      CONFIG_UART_DFU_LIBRARY_THREAD_STACK_SIZE);

static const uart_dfu_cb_t evt_cb[UART_DFU_SESS_COUNT] = {
	[UART_DFU_SESS_IDLE]	= uart_dfu_srv_idle_evt_handle,
	[UART_DFU_SESS_SRV]	= uart_dfu_srv_evt_handle,
	[UART_DFU_SESS_CLI]	= uart_dfu_cli_evt_handle,
	[UART_DFU_SESS_SW]	= uart_dfu_sw_evt_handle,
};


/*****************************************************************************
* Static functions
*****************************************************************************/

static void send_evt_end(enum uart_dfu_evt_type type, int err)
{
	struct uart_dfu_evt evt;

	evt.type = type;
	evt.data.err = err;
	EVT_SEND(&evt);
}

static void sess_enter(void)
{
	struct uart_dfu_evt evt;

	evt.type = UART_DFU_EVT_SESS_ENTER;
	EVT_SEND(&evt);
}

static void sess_exit(enum uart_dfu_sess sess, int err)
{
	struct uart_dfu_evt evt;

	evt.type = UART_DFU_EVT_SESS_EXIT;
	evt.data.err = err;
	evt_cb[sess](&evt);
}

static bool sess_sw_ready(void)
{
	bool ready = true;

	if (atomic_test_bit(state.rx.flags, FLAG_USER_ABORT)) {
		ready = false;
	} else if (atomic_test_bit(state.rx.flags, FLAG_ON)) {
		(void) uart_dfu_rx_stop();
		ready = false;
	} else {
		atomic_clear_bit(state.rx.flags, FLAG_PROCESSING);
		uart_dfu_rx_timeout_stop();
	}

	if (atomic_test_bit(state.tx.flags, FLAG_USER_ABORT)) {
		ready = false;
	} else if (atomic_test_bit(state.tx.flags, FLAG_ON)) {
		(void) uart_dfu_tx_stop();
		ready = false;
	}

	return ready;
}

static void sess_sw_finish(void)
{
	LOG_DBG_DEV("Exiting from session %d", state.sess.pending.from);
	(void) atomic_set(&state.sess.pending.flag, false);
	sess_exit(state.sess.pending.from, state.sess.pending.err);

	LOG_DBG_DEV("Entering session %d", state.sess.pending.to);
	__ASSERT(!atomic_test_bit(state.rx.flags, FLAG_ON) &&
		 !atomic_test_bit(state.tx.flags, FLAG_ON),
		 "Expected RX and TX to be off before entering session");
	(void) atomic_set(&state.sess.current, state.sess.pending.to);
	sess_enter();
}

static int sess_sw_prepare(enum uart_dfu_sess from,
			   enum uart_dfu_sess to, int err)
{
	/* Switch between sessions by passing through a temporary "SW" session
	 * where RX/TX can be stopped and residual events can be safely caught
	 * without interfering with normal sessions.
	 */
	if (!atomic_cas(&state.sess.current, from, UART_DFU_SESS_SW)) {
		return -EBUSY;
	}

	LOG_DBG_DEV("Session switch from %d to %d", from, to);

	state.sess.pending.from	= from;
	state.sess.pending.to	= to;
	state.sess.pending.err	= err;

	(void) atomic_set(&state.sess.pending.flag, true);

	if (sess_sw_ready()) {
		/* Finish synchronously. */
		sess_sw_finish();
		return 0;
	} else {
		/* Finish asynchronously (via uart_dfu_sw_evt_handle()). */
		return -EINPROGRESS;
	}
}

static void uart_dfu_sw_evt_handle(const struct uart_dfu_evt *const evt)
{
	ARG_UNUSED(evt);

	if (atomic_get(&state.sess.pending.flag) && sess_sw_ready()) {
		sess_sw_finish();
	}
}

static int rx_start(size_t len)
{
	(void) atomic_clear(&state.rx.stop_reason);

	int err = uart_rx_enable(state.dev, state.rx.raw.buf,
				 len, SYS_FOREVER_MS);
	if (!err) {
		LOG_DBG_DEV("RX started");
	} else {
		LOG_DBG_DEV("RX failed to start (drv err %d)", err);
	}

	atomic_set_bit_to(state.rx.flags, FLAG_ON, !err);
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
	struct uart_dfu_evt evt;

	evt.type = UART_DFU_EVT_RX;
	evt.data.rx.pdu = (struct uart_dfu_pdu *) state.rx.decoded_buf;

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
		EVT_SEND(&evt);

		if (!atomic_test_bit(state.rx.flags, FLAG_PROCESSING)) {
			/* Processing was aborted due to session switch. */
			ret = -ECANCELED;
			break;
		} else if (atomic_test_and_clear_bit(state.rx.flags, FLAG_ON)) {
			/* uart_dfu_rx_start() was called in event handler. */
			ret = -EAGAIN;
		}
	}

	atomic_clear_bit(state.rx.flags, FLAG_PROCESSING);
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

/*
 * Workqueue handlers
 ****************************************************************************/

static void rx_dis_process(struct k_work *work)
{
	ARG_UNUSED(work);

	atomic_clear_bit(state.rx.flags, FLAG_ON);

	if (atomic_test_and_clear_bit(state.rx.flags, FLAG_USER_ABORT)) {
		LOG_DBG_DEV("RX user stop event");
		atomic_clear_bit(state.rx.flags, FLAG_PROCESSING);
		send_evt_end(UART_DFU_EVT_RX_END, -ECONNABORTED);
		return;
	}

	switch (atomic_get(&state.rx.stop_reason)) {
	case 0:
		/* Process received data before checking for RX timeout.
		 * These two events may coincide. In case a valid PDU has
		 * been buffered, processing should have higher precedence.
		 *
		 * User shall reset the RX timeout on receiving a valid PDU.
		 * Otherwise, the RX timeout flag will remain unchanged,
		 * and the RX timeout event may finally be dispatched.
		 */
		if (atomic_test_bit(state.rx.flags, FLAG_PROCESSING)) {
			rx_process();
		}
		if (atomic_test_bit(state.rx.flags, FLAG_TIMEOUT)) {
			LOG_DBG_DEV("RX timeout event");
			send_evt_end(UART_DFU_EVT_RX_END, -ETIMEDOUT);
		}
		break;
	/* Process UART_RX_STOPPED. */
	case UART_ERROR_OVERRUN:
	case UART_ERROR_PARITY:
	case UART_ERROR_FRAMING:
		/* Malformed data. Reset and resume reception. */
		atomic_clear_bit(state.rx.flags, FLAG_PROCESSING);
		cobs_dec_reset(&state.rx.decoder);
		rx_resume();
		break;
	case UART_BREAK:
		LOG_DBG_DEV("RX break event");
		send_evt_end(UART_DFU_EVT_RX_END, -ENETDOWN);
		break;
	default:
		break;
	}
}

static void tx_dis_process(struct k_work *work)
{
	ARG_UNUSED(work);

	atomic_clear_bit(state.tx.flags, FLAG_ON);

	if (atomic_test_and_clear_bit(state.tx.flags, FLAG_USER_ABORT)) {
		LOG_DBG_DEV("TX user stop event");
		send_evt_end(UART_DFU_EVT_TX_END, -ECONNABORTED);
	} else if (atomic_test_bit(state.tx.flags, FLAG_TIMEOUT)) {
		LOG_DBG_DEV("TX timeout event");
		send_evt_end(UART_DFU_EVT_TX_END, -ETIMEDOUT);
	} else {
		LOG_DBG_DEV("TX complete event");
		send_evt_end(UART_DFU_EVT_TX_END, 0);
	}
}

/*
 * Timer event handlers
 ****************************************************************************/

static void rx_timer_expired(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	if (atomic_test_bit(state.rx.flags, FLAG_ON)) {
		atomic_set_bit(state.rx.flags, FLAG_TIMEOUT);
		(void) uart_rx_disable(state.dev);
	}
}

/*
 * UART event handlers
 ****************************************************************************/

static void uart_tx_handle(struct uart_event_tx *evt, bool aborted)
{
	LOG_DBG_DEV("TX %s (len=%u)", (aborted ? "aborted" : ""), evt->len);

	if (atomic_test_bit(state.tx.flags, FLAG_ON)) {
		if (aborted) {
			/* TX timed out. */
			atomic_set_bit(state.tx.flags, FLAG_TIMEOUT);
		}
	} else {
		/* TX was stopped by user. */
		__ASSERT(aborted, "Expected to be in UART_TX_ABORTED");
	}

	k_work_submit_to_queue(&state.work.q, &state.work.tx_dis);
}

static void uart_rx_rdy_handle(struct uart_event_rx *evt)
{
	__ASSERT(evt->buf == state.rx.raw.buf,
		 "Expected event buf and local buf to be the same");

	LOG_DBG_DEV("RX (offset=%u, len=%u)", evt->offset, evt->len);

	if (!atomic_test_and_set_bit(state.rx.flags, FLAG_PROCESSING)) {
		state.rx.raw.offset = evt->offset;
		state.rx.raw.len = evt->len;
	} else {
		__ASSERT(0, "RX buffer overwritten before it was processed");
	}
}

static void uart_rx_stopped_handle(struct uart_event_rx_stop *evt)
{
	LOG_DBG_DEV("RX stopped (reason=%d)", evt->reason);

	(void) atomic_set(&state.rx.stop_reason, evt->reason);
}

static void uart_rx_disabled_handle(void)
{
	k_work_submit_to_queue(&state.work.q, &state.work.rx_dis);
}

static void uart_async_cb(struct uart_event *evt, void *user_data)
{
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

/*****************************************************************************
* API functions
*****************************************************************************/

static int uart_dfu_sys_init(struct device *dev)
{
	ARG_UNUSED(dev);

	/* Driver init. */
	state.dev = device_get_binding(DT_LABEL(UART_DFU_DT));
	if (state.dev == NULL) {
		return -EACCES;
	}

	int err = uart_callback_set(state.dev, uart_async_cb, NULL);
	if (err) {
		state.dev = NULL;
		return -EIO;
	}

	/* Module init. */
	k_timer_init(&state.rx.timer, rx_timer_expired, NULL);
	k_timer_user_data_set(&state.rx.timer, NULL);

	cobs_dec_init(&state.rx.decoder, state.rx.decoded_buf);

	k_work_init(&state.work.rx_dis, rx_dis_process);
	k_work_init(&state.work.tx_dis, tx_dis_process);

	k_work_q_start(&state.work.q, work_q_stack_area,
		       K_THREAD_STACK_SIZEOF(work_q_stack_area),
		       CONFIG_UART_DFU_LIBRARY_THREAD_PRIO);

	/* Enter idle session. */
	sess_enter();
	return 0;
}

uint8_t *uart_dfu_tx_buf_get(void)
{
	return state.tx.buf.buf;
}

int uart_dfu_tx_start(size_t len, int timeout)
{
	int err;

	if (atomic_test_and_set_bit(state.tx.flags, FLAG_ON)) {
		LOG_DBG_DEV("TX was already started");
		return -EBUSY;
	}

	err = cobs_encode(&state.tx.buf, len);
	if (err) {
		atomic_clear_bit(state.tx.flags, FLAG_ON);
		return err;
	}

	atomic_clear_bit(state.tx.flags, FLAG_TIMEOUT);

	err = uart_tx(state.dev, (uint8_t *) &state.tx.buf,
		      COBS_ENCODED_SIZE(len), timeout);
	if (err) {
		LOG_DBG_DEV("TX failed to start (drv err %d)", err);
		atomic_clear_bit(state.tx.flags, FLAG_ON);
		return err;
	}
	return 0;
}

/* TODO maybe remove */
int uart_dfu_tx_stop(void)
{
	int err = uart_tx_abort(state.dev);
	if (err) {
		return err;
	}

	atomic_clear_bit(state.tx.flags, FLAG_ON);
	atomic_set_bit(state.tx.flags, FLAG_USER_ABORT);
	return 0;
}

int uart_dfu_rx_start(size_t len)
{
	if (atomic_test_and_set_bit(state.rx.flags, FLAG_ON)) {
		LOG_DBG_DEV("RX was already started");
		return -EBUSY;
	}

	/* Generally, do not start RX during buffer processing. */
	if (!atomic_test_bit(state.rx.flags, FLAG_PROCESSING)) {
		int err = rx_start(COBS_ENCODED_SIZE(len));
		if (err) {
			return err;
		}
	} else if (uart_dfu_in_work_q_thread()) {
		/* RX will start after buffer processing finishes. */
	} else {
		LOG_DBG_DEV("RX failed to start (processing)");
		atomic_clear_bit(state.rx.flags, FLAG_ON);
		return -EBUSY;
	}

	state.rx.expected_len = COBS_ENCODED_SIZE(len);
	return 0;
}

int uart_dfu_rx_stop(void)
{
	int err = uart_rx_disable(state.dev);
	if (err) {
		return err;
	}

	atomic_clear_bit(state.rx.flags, FLAG_ON);
	atomic_set_bit(state.rx.flags, FLAG_USER_ABORT);
	return 0;
}

void uart_dfu_rx_timeout_start(int timeout)
{
	LOG_DBG_DEV("Starting timeout: %d ms", timeout);

	k_timer_start(&state.rx.timer, K_MSEC(timeout), K_NO_WAIT);
	atomic_clear_bit(state.rx.flags, FLAG_TIMEOUT);
}

void uart_dfu_rx_timeout_stop(void)
{
	k_timer_stop(&state.rx.timer);
	atomic_clear_bit(state.rx.flags, FLAG_TIMEOUT);
}

int uart_dfu_sess_open(enum uart_dfu_sess sess)
{
	if (sess != UART_DFU_SESS_SRV && sess != UART_DFU_SESS_CLI) {
		return -EINVAL;
	}

	if (atomic_get(&state.sess.current) == sess) {
		return -EALREADY;
	} else {
		return sess_sw_prepare(UART_DFU_SESS_IDLE, sess, 0);
	}
}

int uart_dfu_sess_close(enum uart_dfu_sess sess, int err)
{
	if (sess != UART_DFU_SESS_SRV && sess != UART_DFU_SESS_CLI) {
		return -EINVAL;
	}

	return sess_sw_prepare(sess, UART_DFU_SESS_IDLE, err);
}

bool uart_dfu_in_work_q_thread(void)
{
	return k_current_get() == &state.work.q.thread;
}

/*****************************************************************************
* System initialization hooks
*****************************************************************************/

SYS_INIT(uart_dfu_sys_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
