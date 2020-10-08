/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_COBS_H__
#define UART_COBS_H__

#include <stddef.h>
#include <zephyr.h>
#include <zephyr/types.h>

/**
 * @file uart_cobs.h
 * @defgroup uart_cobs UART Consistent Overhead Byte Stuffing (COBS) library.
 * @{
 * @brief UART Consistent Overhead Byte Stuffing (COBS) library API.
 */

/** Maximum payload size for UART COBS packets */
#define UART_COBS_MAX_PAYLOAD_LEN 253

/**
 * @brief Define UART COBS user structure.
 */
#define UART_COBS_USER_DEFINE(_name, _cb, _ctx)	\
	static struct uart_cobs_user _name = {	\
		.cb = _cb,			\
		.ctx = _ctx			\
	}

/**
 * @brief UART COBS event types.
 */
enum uart_cobs_evt_type {
	/** UART access granted to user. */
	UART_COBS_EVT_USER_START,
	/** UART access revoked from user. */
	UART_COBS_EVT_USER_END,
	/** Received data. */
	UART_COBS_EVT_RX,
	/** Reception aborted by user/timeout/UART break. */
	UART_COBS_EVT_RX_END,
	/** Transmission completed or aborted by user/timeout. */
	UART_COBS_EVT_TX_END,
};

enum uart_cobs_end_reason {
	UART_COBS_END_REASON_DONE	= 0,
	UART_COBS_END_REASON_ABORT	= -ECONNABORTED,
	UART_COBS_END_REASON_TIMEOUT	= -ETIMEDOUT,
	UART_COBS_END_REASON_BREAK	= -ENETDOWN
};

/**
 * @brief UART COBS event structure.
 */
struct uart_cobs_evt {
	/** Event type. */
	enum uart_cobs_evt_type type;
	/** Event data. */
	union {
		/** Event structure used by UART_COBS_EVT_RX. */
		struct {
			const uint8_t *buf;
			size_t len;
		} rx;
		/**
		 * Error value used by UART_COBS_EVT_USER_EXIT,
		 * UART_COBS_EVT_RX_END, and UART_COBS_EVT_TX_END.
		 */
		int err;
	} data;
};

/**
 * @brief UART COBS user structure.
 */
struct uart_cobs_user {
	void (*cb)(const struct uart_cobs_evt *const evt, void *ctx);
	void *ctx;
};

/**
 * @brief Set user event handler and switch to user.
 * @param user_cb Event handler.
 * @details This function should only be called when no user is active.
 *          An event with type @ref UART_COBS_EVT_USER_START will be generated
 *          and sent to the given @p user_cb if the handler was successfully
 *          set.
 * @retval 0 Switched to user synchronously.
 * @retval -EINPROGRESS Started asynchronous switch to user.
 * @retval -EINVAL @p user_cb was NULL. 
 * @retval -EBUSY Another user is already active (i.e. not in the idle state).
 * @retval -EALREADY @p user_cb is already active.
 */
int uart_cobs_user_start(struct uart_cobs_user *user);

/**
 * @brief Clear user event handler and switch to idle state.
 * @param user_cb Event handler.
 * @details An event with type @ref UART_COBS_EVT_USER_END will be generated
 *          and sent to the given @p user_cb when the user is disabled.
 * @param err Error code to be passed to event handler.
 * @retval 0 Switched to the idle state synchronously.
 * @retval -EINPROGRESS Started asynchronous switch to the idle state.
 * @retval -EINVAL @p user_cb was NULL. 
 * @retval -EBUSY The given user is not active.
 */
int uart_cobs_user_end(struct uart_cobs_user *user, int err);

bool uart_cobs_user_active(struct uart_cobs_user *user);

/**
 * @brief Set event handler for the idle state (i.e. when no user is active).
 * @details This function should only be called when no user is active.
 *          An event with type @ref UART_COBS_EVT_USER_START will be generated
 *          and sent to the given @p idle_cb if the handler was successfully
 *          set.
 * @param idle_cb Event handler. Set to NULL to clear existing handler.
 * @retval 0 Successfully set idle event handler.
 * @retval -EBUSY Can not update idle handler in the current state.
 */
int uart_cobs_default_user_set(struct uart_cobs_user *user);

/**
 * @brief Get a pointer to the UART COBS TX buffer.
 * @details Data written to the returned buffer will be automatically
 *          COBS-encoded and transmitted by calling @ref uart_cobs_tx_start.
 *          The returned buffer has max. length of @ref COBS_MAX_DATA_BYTES.
 * @returns Pointer to the TX buffer.
 */

int uart_cobs_tx_buf_write(struct uart_cobs_user *user,
			   uint8_t *data, size_t len);

int uart_cobs_tx_buf_clear(struct uart_cobs_user *user);

/**
 * @brief Start transmission of the UART COBS TX buffer.
 * @details Data should be written directly to the buffer returned by
 *          @ref uart_cobs_tx_buf_get prior to calling this function.
 * @param len      Length of data in the TX buffer (unencoded).
 * @param timeout  Timeout in milliseconds.
 * @retval 0       Successfully started transmission.
 * @retval -EBUSY  Transmission already started.
 */
int uart_cobs_tx_start(struct uart_cobs_user *user, int timeout);

/**
 * @brief Stop ongoing transmission.
 * @details The transmission is aborted asynchronously. An event with type
 *          @ref UART_COBS_EVT_TX_END will be generated once stopped.
 * @retval 0 Stopping the transmission.
 */
int uart_cobs_tx_stop(struct uart_cobs_user *user);

/**
 * @brief Start reception of @p len bytes of data.
 * @param len Length of data to receive (unencoded).
 * @retval 0 Successfully started reception.
 * @retval -EBUSY Reception already started. 
 */
int uart_cobs_rx_start(struct uart_cobs_user *user, size_t len);

/**
 * @brief Stop ongoing reception.
 * @details The reception is aborted asynchronously. An event with type
 *          @ref UART_COBS_EVT_RX_END will be generated once stopped.
 * @retval 0 Stopping the reception.
 */
int uart_cobs_rx_stop(struct uart_cobs_user *user);

/**
 * @brief Start RX timeout.
 * @param timeout Timeout in milliseconds.
 */
int uart_cobs_rx_timeout_start(struct uart_cobs_user *user, int timeout);

/**
 * @brief Stop RX timeout.
 */
int uart_cobs_rx_timeout_stop(struct uart_cobs_user *user);

/**
 * @brief Check if currently in the UART COBS workqueue thread context.
 * @returns true if in the workqueue thread, false otherwise.
 */
bool uart_cobs_in_work_q_thread(void);

/** @} */

#endif  /* UART_COBS_H__ */
