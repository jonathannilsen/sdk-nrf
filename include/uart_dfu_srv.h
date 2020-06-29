/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_DFU_SRV_H__
#define UART_DFU_SRV_H__

#include <uart_dfu.h>
#include <zephyr.h>

/**
 * @file uart_dfu_srv.h
 * @defgroup uart_dfu_srv UART DFU server.
 * @{
 * @brief UART DFU server API.
 */

/*****************************************************************************
 * Structure definitions
 *****************************************************************************/

/**
 * @brief UART DFU server event type.
 */
enum uart_dfu_srv_evt_type {
	/**
	 * @brief Server session started by remote client.
	 */
	UART_DFU_SRV_EVT_STARTED = 0,

	/**
	 * @brief Server session stopped either due to being finished or
	 *        due to a protocol error.
	 */
	UART_DFU_SRV_EVT_STOPPED,

	/**
	 * @brief Number of server events.
	 */
	UART_DFU_SRV_EVT_COUNT
};

/**
 * @brief UART DFU server protocol events.
 */
enum uart_dfu_srv_err {
	/**
	 * @brief Server session finished normally.
	 */
	UART_DFU_SRV_SUCCESS 		= 0,

	/**
	 * @brief Server session aborted by user.
	 */
	UART_DFU_SRV_ERR_ABORT		= -ECONNABORTED,

	/**
	 * @brief Server session aborted due to timeout.
	 */
	UART_DFU_SRV_ERR_TIMEOUT	= -ETIMEDOUT,

	/**
	 * @brief Server session aborted due to UART break error.
	 */
	UART_DFU_SRV_ERR_BREAK		= -ENETDOWN,

	/**
	 * @brief Server session aborted due to unhandled fatal error.
	 */
	UART_DFU_SRV_ERR_FATAL		= -EFAULT
};

/**
 * @brief UART DFU server event structure.
 */
struct uart_dfu_srv_evt {
	/**
	 * @brief Type of event.
	 */
	enum uart_dfu_srv_evt_type type;

	/**
	 * @brief Error description.
	 */
	enum uart_dfu_srv_err err;
};

/**
 * @brief UART DFU server callback structure.
 */
struct uart_dfu_srv_cb {
	/**
	 * @brief Callback to signal initialized file transfer from the remote
	 * 	  client.
	 * @param[in] file_size Size of the file to be transferred.
	 * @returns 0 if successfully handled, otherwise negative error code.
	 */
	int (*init_cb)(size_t file_size);

	/**
	 * @brief Callback to signal received file fragment from the remote
	 * 	  client.
	 * @param[in] buf Buffer containing file fragment.
	 * @param[in] len Length of file fragment.
	 * @returns 0 if successfully handled, otherwise negative error code.
	 */
	int (*write_cb)(const uint8_t *buf, size_t len);

	/**
	 * @brief Callback to signal a file offset request from the remote
	 * 	  client.
	 * @param[out] offset Returns the offset of the file transfer.
	 * @returns 0 if successfully handled, otherwise negative error code.
	 */
	int (*offset_cb)(size_t *offset);

	/**
	 * @brief Callback to signal finished file transfer from the remote
	 * 	  client.
	 * @param[in] successful Indicates whether the file transfer completed
	 * 			 successfully or was aborted.
	 * @returns 0 if successfully handled, otherwise negative error code.
	 */
	int (*done_cb)(bool successful);

	/**
	 * @brief Callback to signal UART DFU server protocol event.
	 * @param[in] evt Server protocol event.
	 */
	void (*evt_cb)(const struct uart_dfu_srv_evt *const evt);
};

/*****************************************************************************
 * UART DFU API functions
 *****************************************************************************/

/**
 * @brief UART DFU server event handler.
 * @param[in] evt UART DFU protocol event.
 */
void uart_dfu_srv_evt_handle(const struct uart_dfu_evt *const evt);

/**
 * @brief UART DFU idle event handler.
 * @param[in] evt UART DFU protocol event.
 */
void uart_dfu_srv_idle_evt_handle(const struct uart_dfu_evt *const evt);


/*****************************************************************************
 * Public API functions
 *****************************************************************************/

/**
 * @brief Initialize UART DFU server.
 * @param[in] buf Buffer to store received file fragments.
 * @param[in] max_len Maximum length of fragment to store in @p buf.
 * @param[in] callbacks Application event callback functions.
 * @retval 0 If successful.
 * @retval -EINVAL If @p buf given was NULL, @p max_len given was 0 or
 * 		   @p callbacks or any of its members was NULL.
 */
int uart_dfu_srv_init(uint8_t *buf,
		      size_t max_len,
		      struct uart_dfu_srv_cb *callbacks);

/**
 * @brief Start UART DFU server to enable file transfer.
 */
void uart_dfu_srv_enable(void);

/**
 * @brief Stop UART DFU server, aborting any ongoing actions.
 * @retval 0 If there were no actions to be aborted.
 * @retval -EINPROGRESS If there were actions to be aborted. The application
 * 			should wait for an asynchronous error callback before
 * 			continuing.
 */
int uart_dfu_srv_disable(void);

/** @} */

#endif // UART_DFU_SRV_H__
