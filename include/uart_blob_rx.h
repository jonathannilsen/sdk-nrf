/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_BLOB_RX_H__
#define UART_BLOB_RX_H__

#include <zephyr.h>
#include <uart_blob.h>

/**
 * @file uart_blob_rx.h
 * @defgroup uart_blob_rx UART BLOB receiver.
 * @{
 * @brief UART BLOB receiver API.
 */

/**
 * @brief UART BLOB receiver callback structure.
 */
struct uart_blob_rx_cb {
	/**
	 * @brief Callback to signal initialized BLOB transfer from the remote
	 * 	  sender.
	 * @param[in] blob_len Size of the BLOB to be transferred.
	 * @returns 0 if successfully handled, otherwise negative error code.
	 */
	int (*init_cb)(size_t blob_len);

	/**
	 * @brief Callback to signal received BLOB fragment from the remote
	 * 	  sender.
	 * @param[in] buf Buffer containing BLOB fragment.
	 * @param[in] len Length of BLOB fragment.
	 * @returns 0 if successfully handled, otherwise negative error code.
	 */
	int (*write_cb)(const uint8_t *buf, size_t len);

	/**
	 * @brief Callback to signal a BLOB offset request from the remote
	 * 	  sender.
	 * @param[out] offset The offset of the BLOB transfer.
	 *                    The handler should set this value to the current
	 *                    offset of the transfer.
	 * @returns 0 if successfully handled, otherwise negative error code.
	 */
	int (*offset_cb)(size_t *offset);

	/**
	 * @brief Callback to signal finished BLOB transfer from the remote
	 * 	  sender.
	 * @param[in] successful Indicates whether the BLOB transfer completed
	 * 			 successfully or was aborted.
	 * @returns 0 if successfully handled, otherwise negative error code.
	 */
	int (*done_cb)(bool successful);

	/**
	 * @brief Callback to signal UART BLOB receiver protocol event.
	 * @param[in] evt Receiver protocol event.
	 */
	void (*evt_cb)(const struct uart_blob_evt *const evt);
};

/**
 * @brief Initialize UART BLOB receiver.
 * @param[in] buf Buffer to store received BLOB fragments.
 * @param[in] max_len Maximum length of fragment to store in @p buf.
 * @param[in] callbacks Application event callback functions.
 * @retval 0 If successful.
 * @retval -EINVAL If @p buf given was NULL, @p max_len given was 0 or
 * 		   @p callbacks or any of its members was NULL.
 */
int uart_blob_rx_init(uint8_t *buf,
		      size_t max_len,
		      struct uart_blob_rx_cb *callbacks);

/**
 * @brief Start UART BLOB receiver to enable BLOB transfer.
 */
void uart_blob_rx_enable(void);

/**
 * @brief Stop UART BLOB receiver, aborting any ongoing actions.
 * @retval 0 If there were no actions to be aborted.
 * @retval -EINPROGRESS If there were actions to be aborted. The application
 * 			should wait for an asynchronous error callback before
 * 			continuing.
 */
int uart_blob_rx_disable(void);

/** @} */

#endif // UART_BLOB_RX_H__
