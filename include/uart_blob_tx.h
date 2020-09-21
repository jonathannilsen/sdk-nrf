/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_BLOB_TX_H__
#define UART_BLOB_TX_H__

#include <uart_cobs.h>
#include <zephyr.h>


/**
 * @file uart_blob_tx.h
 * @defgroup uart_blob_tx UART BLOB sender.
 * @{
 * @brief UART BLOB sender API.
 */


/*****************************************************************************
 * Structure definitions
 *****************************************************************************/

/**
 * @brief UART BLOB sender event type.
 */
enum uart_blob_tx_evt_type {
	/**
	 * @brief BLOB transfer initiated.
	 */
	UART_BLOB_TX_EVT_STARTED = 0,

	/**
	 * @brief BLOB transfer stopped either due to being finished or
	 *        due to a protocol error.
	 */
	UART_BLOB_TX_EVT_STOPPED,

	/**
	 * @brief Number of sender events.
	 */
	UART_BLOB_TX_EVT_COUNT
};

/**
 * @brief UART BLOB sender protocol events.
 */
enum uart_blob_tx_err {
	/**
	 * @brief BLOB transfer closed by user.
	 */
	UART_BLOB_TX_SUCCESS 		= 0,

	/**
	 * @brief BLOB transfer aborted due to timeout.
	 */
	UART_BLOB_TX_ERR_TIMEOUT	= -ETIMEDOUT,

	/**
	 * @brief BLOB transfer aborted due to UART break error.
	 */
	UART_BLOB_TX_ERR_BREAK		= -ENETDOWN,

	/**
	 * @brief BLOB transfer aborted due to unhandled fatal error.
	 */
	UART_BLOB_TX_ERR_FATAL		= -EFAULT
};

/**
 * @brief UART BLOB sender event structure.
 */
struct uart_blob_tx_evt {
	/**
	 * @brief Type of event.
	 */
	enum uart_blob_tx_evt_type type;

	/**
	 * @brief Error description.
	 */
	enum uart_blob_tx_err err;
};

/**
 * @brief UART BLOB sender callback structure.
 */
struct uart_blob_tx_cb {
	/**
	 * @brief Callback to signal received status from the remote receiver.
	 * @param[in] status Status code returned by the remote receiver.
	 */
	void (*status_cb)(int status);

	/**
	 * @brief Callback to signal received offset from the remote receiver.
	 * @param[in] offset Offset returned by the remote receiver.
	 */
	void (*offset_cb)(size_t offset);

	/**
	 * @brief Callback to signal UART BLOB sender protocol event.
	 * @param[in] evt Client protocol event.
	 */
	void (*evt_cb)(const struct uart_blob_tx_evt *const evt);
};


/*****************************************************************************
 * Public API functions
 *****************************************************************************/

/**
 * @brief Initialize UART BLOB sender.
 * @param[in] callbacks Application event callback functions.
 * @retval 0 If successful.
 * @retval -EINVAL If either @p callbacks or one of its members was NULL.
 */
int uart_blob_tx_init(struct uart_blob_tx_cb *callbacks);

/**
 * @brief Start UART BLOB sender to enable BLOB transfer.
 * @retval 0 If successful.
 * @retval -EBUSY If unable to start due to other UART DFU activity.
 */
int uart_blob_tx_enable(void);

/**
 * @brief Stop UART BLOB sender, aborting any ongoing actions.
 * @retval 0 If there were no actions to be aborted.
 * @retval -EINPROGRESS If there were actions to be aborted. The application
 * 			should wait for an asynchronous error callback before
 * 			continuing.
 */
int uart_blob_tx_disable(void);

/**
 * @brief Send a message to the remote receiver to initialize BLOB transfer.
 * @param[in] blob_len Size of the BLOB to be transferred.
 * @retval 0 If successful.
 * @retval -ENOMEM If @p blob_len given was too large.
 * @retval -EBUSY If unable to start the transmission.
 */
int uart_blob_tx_send_init(size_t blob_len);

/**
 * @brief Send a BLOB fragment to the remote receiver.
 * @param[in] buf Buffer containing BLOB fragment.
 * @param[in] len Length of BLOB fragment.
 * @retval 0 If successful.
 * @retval -EINVAL If @p buf given was NULL or @p len given was 0.
 * @retval -ENOMEM If @p len given was too large.
 * @retval -EBUSY If unable to start the transmission.
 */
int uart_blob_tx_send_write(const uint8_t *const buf, size_t len);

/**
 * @brief Request BLOB offset from the remote receiver.
 * @retval 0 If successful.
 * @retval -EBUSY If unable to start the transmission.
 */
int uart_blob_tx_send_offset(void);

/**
 * @brief Send a message to the remote receiver to finish BLOB transfer.
 * @param[in] successful Indicate whether the BLOB transfer completed
 * 			 successfully or was aborted.
 * @retval 0 If successful.
 * @retval -EBUSY If unable to start the transmission.
 */
int uart_blob_tx_send_done(bool successful);


/** @} */

#endif // UART_BLOB_TX_H__
