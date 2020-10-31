/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_BLOB_H__
#define UART_BLOB_H__

#include <errno.h>

/**
 * @file uart_blob.h
 * @defgroup uart_blob UART Binary Large OBject (BLOB) common types.
 * @{
 * @brief UART Binary Large OBject (BLOB) common types.
 */

/**
 * @brief UART BLOB event type.
 */
enum uart_blob_evt_type {
	/**
	 * @brief BLOB transfer initiated.
	 */
	UART_BLOB_EVT_STARTED = 0,

	/**
	 * @brief BLOB transfer stopped either due to being finished or
	 *        due to a protocol error.
	 */
	UART_BLOB_EVT_STOPPED,

	/**
	 * @brief Number of UART BLOB events.
	 */
	UART_BLOB_EVT_COUNT
};

/**
 * @brief UART BLOB protocol events.
 */
enum uart_blob_status {
	/**
	 * @brief BLOB transfer closed by user.
	 */
	UART_BLOB_SUCCESS	= 0,
	
	/**
	 * @brief BLOB transfer aborted by user.
	 */
	UART_BLOB_ERR_ABORT	= -ECONNABORTED,

	/**
	 * @brief BLOB transfer aborted due to timeout.
	 */
	UART_BLOB_ERR_TIMEOUT	= -ETIMEDOUT,

	/**
	 * @brief BLOB transfer aborted due to UART break error.
	 */
	UART_BLOB_ERR_BREAK	= -ENETDOWN,

	/**
	 * @brief BLOB transfer aborted due to unhandled fatal error.
	 */
	UART_BLOB_ERR_FATAL	= -EFAULT
};

/**
 * @brief UART BLOB sender event structure.
 */
struct uart_blob_evt {
	/**
	 * @brief Type of event.
	 */
	enum uart_blob_evt_type type;

	/**
	 * @brief Status description.
	 */
	enum uart_blob_status status;
};

/** @} */

#endif // UART_BLOB_H__
