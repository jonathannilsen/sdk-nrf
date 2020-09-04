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

/* TODO:
	- Put checks in static inline
	- Rename api
	- Decouple callback api
*/

/*****************************************************************************
* Macros
*****************************************************************************/

#define EVT_SEND	 ((uart_dfu_cb_t) atomic_ptr_get(&state.user.current))

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
* Forward declarations
*****************************************************************************/

static void sw_evt_handle(const struct uart_dfu_evt *const evt);


/*****************************************************************************
* Static variables
*****************************************************************************/

enum op_status {
	STATUS_OFF,
	STATUS_ON,
	STATUS_ERR
};

struct {
	struct device *dev;
	struct {
		atomic_t status;
		int err;
		struct cobs_enc_buf buf;
	} tx;
	struct {
		atomic_t status;
		int err;
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
		uart_dfu_cb_t idle;
		struct {
			uart_dfu_cb_t from;
			uart_dfu_cb_t to;
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
		      CONFIG_UART_DFU_LIBRARY_THREAD_STACK_SIZE);


/*****************************************************************************
* Static functions
*****************************************************************************/

bool status_error_set(atomic_t *status, int *status_err, int err)
{
	if (atomic_cas(status, STATUS_ON, STATUS_ERR)) {
		*status_err = err;
		return true;
	} else {
		return false;
	}
}

static inline bool rx_error_set(int err)
{
	return status_error_set(&state.rx.status, &state.rx.err, err);
}

static inline bool tx_error_set(int err)
{
	return status_error_set(&state.tx.status, &state.tx.err, err);
}

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

static void sess_exit(uart_dfu_cb_t from, int err)
{
	if (from != NULL) {
		struct uart_dfu_evt evt;
		evt.type = UART_DFU_EVT_SESS_EXIT;
		evt.data.err = err;
		from(&evt);
	}
}

static bool user_sw_ready(void)
{
	bool ready = true;

	/* Check current RX status. */
	switch (atomic_get(&state.rx.status)) {
	case STATUS_ON:
		(void) uart_dfu_rx_stop();
		ready = false;
		break;
	case STATUS_ERR:
		ready = false;
		break;
	case STATUS_OFF:
	default:
		state.rx.processing = false; // TODO: necessary?
		uart_dfu_rx_timeout_stop();
		break;
	}

	/* Check current TX status. */
	switch (atomic_get(&state.tx.status)) {
	case STATUS_ON:
		(void) uart_dfu_tx_stop();
		ready = false;
		break;
	case STATUS_ERR:
		ready = false;
		break;
	case STATUS_OFF:
	default:
		break;
	}

	return ready;
}

static void user_sw_finish(void)
{
	LOG_DBG_DEV("Exiting from session %d", state.user.pending.from);
	sess_exit(state.user.pending.from, state.user.pending.err);

	LOG_DBG_DEV("Entering session %d", state.user.pending.to);
	__ASSERT(atomic_get(&state.rx.status) == STATUS_OFF &&
		 atomic_get(&state.tx.status) == STATUS_OFF,
		 "Expected RX and TX to be off before entering session");
	(void) atomic_ptr_set(&state.user.current, state.user.pending.to);
	sess_enter();
}

static int user_sw_prepare(uart_dfu_cb_t from, uart_dfu_cb_t to, int err)
{
	/* Switch between sessions by passing through a temporary "SW" session
	 * where RX/TX can be stopped and residual events can be safely caught
	 * without interfering with normal sessions.
	 */

	if (!atomic_ptr_cas(&state.user.current, from, sw_evt_handle)) {
		return -EBUSY;
	}

	LOG_DBG_DEV("Session switch from %lu to %lu",
		(void *) from, (void *) to);

	state.user.pending.from	= from;
	state.user.pending.to	= to;
	state.user.pending.err	= err;

	if (user_sw_ready()) {
		/* Finish synchronously. */
		user_sw_finish();
		return 0;
	} else {
		/* Finish asynchronously (via sw_evt_handle()). */
		return -EINPROGRESS;
	}
}

static void sw_evt_handle(const struct uart_dfu_evt *const evt)
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
	struct uart_dfu_evt evt;

	evt.type = UART_DFU_EVT_RX;
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
		EVT_SEND(&evt);

		if (!state.rx.processing) {
			/* Processing was aborted due to session switch. */
			ret = -ECANCELED;
			break;
		} else if (atomic_get(&state.rx.status) == STATUS_ON) {
			/* uart_dfu_rx_start() was called in event handler. */
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

/*
 * Workqueue handlers
 ****************************************************************************/

static void rx_dis_process(struct k_work *work)
{
	ARG_UNUSED(work);

	enum op_status status;
	status = (enum op_status) atomic_set(&state.rx.status, STATUS_OFF);

	switch (status) {
	case STATUS_ON:
		if (state.rx.processing) {
			rx_process();
		}
		break;
	case STATUS_ERR:
		if (state.rx.err == -EAGAIN) {
			/* Malformed data. Reset and resume reception. */
			state.rx.processing = false;
			cobs_dec_reset(&state.rx.decoder);
			rx_resume();
		} else {
			/* Fatal error. */
			LOG_DBG_DEV("RX stopped with error: %d", state.rx.err);
			state.rx.processing = false;  // TODO: necessary?
			send_evt_end(UART_DFU_EVT_RX_END, state.rx.err);
		}
		break;
	case STATUS_OFF:
	default:
		break;
	}
}

static void tx_dis_process(struct k_work *work)
{
	ARG_UNUSED(work);

	enum op_status status;
	status = (enum op_status) atomic_set(&state.tx.status, STATUS_OFF);

	if (status == STATUS_ERR) {
		LOG_DBG_DEV("TX stopped with error %d", state.tx.err);
		send_evt_end(UART_DFU_EVT_TX_END, state.tx.err);
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

	if (rx_error_set(-ETIMEDOUT)) {
		(void) uart_rx_disable(state.dev);
	}
}

/*
 * UART event handlers
 ****************************************************************************/

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
		(void) rx_error_set(-EAGAIN); // FIXME: proper error?	
		break;
	case UART_BREAK:
	default:
		/* FIXME: check that we can't end in an invalid state if
			other errors are always received before this one. */
		(void) rx_error_set(-ENETDOWN);
		break;
	}
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

	if (!atomic_cas(&state.tx.status, STATUS_OFF, STATUS_ON)) {
		LOG_DBG_DEV("TX was already started");
		return -EBUSY;
	}

	err = cobs_encode(&state.tx.buf, len);
	if (err) {
		(void) atomic_set(&state.tx.status, STATUS_OFF);
		return err;
	}

	err = uart_tx(state.dev, (uint8_t *) &state.tx.buf,
		      COBS_ENCODED_SIZE(len), timeout);
	if (err) {
		LOG_DBG_DEV("TX failed to start (drv err %d)", err);
		(void) atomic_set(&state.tx.status, STATUS_OFF);
		return err;
	}
	return 0;
}

/* TODO maybe remove */
int uart_dfu_tx_stop(void)
{
	if (tx_error_set(-ECONNABORTED)) {
		return uart_tx_abort(state.dev);
	}
	return 0;
}

int uart_dfu_rx_start(size_t len)
{
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
	} else if (uart_dfu_in_work_q_thread()) {
		/* RX will start after buffer processing finishes. */
	} else {
		LOG_DBG_DEV("RX failed to start (processing)");
		(void) atomic_set(&state.rx.status, STATUS_OFF);
		return -EBUSY;
	}

	state.rx.expected_len = COBS_ENCODED_SIZE(len);
	return 0;
}

int uart_dfu_rx_stop(void)
{
	if (rx_error_set(-ECONNABORTED)) {
		return uart_rx_disable(state.dev);
	}
	return 0;
}

void uart_dfu_rx_timeout_start(int timeout)
{
	LOG_DBG_DEV("Starting timeout: %d ms", timeout);
	k_timer_start(&state.rx.timer, K_MSEC(timeout), K_NO_WAIT);
}

void uart_dfu_rx_timeout_stop(void)
{
	k_timer_stop(&state.rx.timer);
}

int uart_dfu_user_request(uart_dfu_cb_t user_cb)
{
	if (user_cb == NULL) {
		return -EINVAL;
	}

	if (atomic_ptr_get(&state.user.current) == user_cb) {
		return -EALREADY;
	} else {
		return user_sw_prepare(idle_cb, user_cb, 0);
	}
}

int uart_dfu_user_release(uart_dfu_cb_t user_cb, int err)
{
	return user_sw_prepare(user_cb, idle_cb, err);
}

bool uart_dfu_in_work_q_thread(void)
{
	return k_current_get() == &state.work.q.thread;
}

/*****************************************************************************
* System initialization hooks
*****************************************************************************/

SYS_INIT(uart_dfu_sys_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
