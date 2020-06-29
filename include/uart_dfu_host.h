/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_DFU_HOST_H__
#define UART_DFU_HOST_H__

#include <zephyr.h>
#include <sys/atomic.h>
#include <uart_dfu.h>

/**
 * @file uart_dfu_host.h
 * @defgroup uart_dfu_host UART DFU host.
 * @{
 * @brief UART DFU host API.
 */

/*****************************************************************************
 * API functions
 *****************************************************************************/

/**
 * @brief Initialize the UART DFU host.
 */
void uart_dfu_host_init(void);

/**
 * @brief Enable the UART DFU host to enable file transfer.
 */
void uart_dfu_host_enable(void);

/**
 * @brief Disable the UART DFU host, aborting any ongoing actions.
 */
void uart_dfu_host_disable(void);


/** @} */

#endif /* UART_DFU_HOST_H__ */
