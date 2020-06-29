/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_DFU_CLI_H__
#define UART_DFU_CLI_H__

#include <uart_dfu.h>
#include <zephyr.h>


/**
 * @file uart_dfu_cli.h
 * @defgroup uart_dfu_cli UART DFU client.
 * @{
 * @brief UART DFU client API.
 */


/*****************************************************************************
 * Structure definitions
 *****************************************************************************/

/**
 * @brief UART DFU client event type.
 */
enum uart_dfu_cli_evt_type {
	/**
	 * @brief Client session started by remote client.
	 */
	UART_DFU_CLI_EVT_STARTED = 0,

	/**
	 * @brief Client session stopped either due to being finished or
	 *        due to a protocol error.
	 */
	UART_DFU_CLI_EVT_STOPPED,

	/**
	 * @brief Number of client events.
	 */
	UART_DFU_CLI_EVT_COUNT
};

/**
 * @brief UART DFU client protocol events.
 */
enum uart_dfu_cli_err {
	/**
	 * @brief Client session closed by user.
	 */
	UART_DFU_CLI_SUCCESS 		= 0,

	/**
	 * @brief Client session aborted due to timeout.
	 */
	UART_DFU_CLI_ERR_TIMEOUT	= -ETIMEDOUT,

	/**
	 * @brief Client session aborted due to UART break error.
	 */
	UART_DFU_CLI_ERR_BREAK		= -ENETDOWN,

	/**
	 * @brief Client session aborted due to unhandled fatal error.
	 */
	UART_DFU_CLI_ERR_FATAL		= -EFAULT
};

/**
 * @brief UART DFU client event structure.
 */
struct uart_dfu_cli_evt {
	/**
	 * @brief Type of event.
	 */
	enum uart_dfu_cli_evt_type type;

	/**
	 * @brief Error description.
	 */
	enum uart_dfu_cli_err err;
};

/**
 * @brief UART DFU client callback structure.
 */
struct uart_dfu_cli_cb {
	/**
	 * @brief Callback to signal received status from the remote server.
	 * @param[in] status Status code returned by the remote server.
	 */
	void (*status_cb)(int status);

	/**
	 * @brief Callback to signal received offset from the remote server.
	 * @param[in] offset Offset returned by the remote server.
	 */
	void (*offset_cb)(size_t offset);

	/**
	 * @brief Callback to signal UART DFU client protocol event.
	 * @param[in] evt Client protocol event.
	 */
	void (*evt_cb)(const struct uart_dfu_cli_evt *const evt);
};


/*****************************************************************************
 * UART DFU API functions
 *****************************************************************************/

/**
 * @brief UART DFU client event handler
 * @param[in] evt UART DFU protocol event.
 */
void uart_dfu_cli_evt_handle(const struct uart_dfu_evt *const evt);


/*****************************************************************************
 * Public API functions
 *****************************************************************************/

/**
 * @brief Initialize UART DFU client.
 * @param[in] callbacks Application event callback functions.
 * @retval 0 If successful.
 * @retval -EINVAL If either @p callbacks or one of its members was NULL.
 */
int uart_dfu_cli_init(struct uart_dfu_cli_cb *callbacks);

/**
 * @brief Start UART DFU client to enable file transfer.
 * @retval 0 If successful.
 * @retval -EBUSY If unable to start due to other UART DFU activity.
 */
int uart_dfu_cli_start(void);

/**
 * @brief Stop UART DFU client, aborting any ongoing actions.
 * @retval 0 If there were no actions to be aborted.
 * @retval -EINPROGRESS If there were actions to be aborted. The application
 * 			should wait for an asynchronous error callback before
 * 			continuing.
 */
int uart_dfu_cli_stop(void);

/**
 * @brief Send a message to the remote server to initialize file transfer.
 * @param[in] file_size Size of the file to be transferred.
 * @retval 0 If successful.
 * @retval -ENOMEM If @p file_size given was too large.
 * @retval -EBUSY If unable to start the transmission.
 */
int uart_dfu_cli_init_send(size_t file_size);

/**
 * @brief Send a file fragment to the remote server.
 * @param[in] buf Buffer containing file fragment.
 * @param[in] len Length of file fragment.
 * @retval 0 If successful.
 * @retval -EINVAL If @p buf given was NULL or @p len given was 0.
 * @retval -ENOMEM If @p len given was too large.
 * @retval -EBUSY If unable to start the transmission.
 */
int uart_dfu_cli_write_send(const uint8_t *const buf, size_t len);

/**
 * @brief Request file offset from the remote server.
 * @retval 0 If successful.
 * @retval -EBUSY If unable to start the transmission.
 */
int uart_dfu_cli_offset_send(void);

/**
 * @brief Send a message to the remote server to finish file transfer.
 * @param[in] successful Indicate whether the file transfer completed
 * 			 successfully or was aborted.
 * @retval 0 If successful.
 * @retval -EBUSY If unable to start the transmission.
 */
int uart_dfu_cli_done_send(bool successful);


/** @} */

#endif // UART_DFU_CLI_H__
