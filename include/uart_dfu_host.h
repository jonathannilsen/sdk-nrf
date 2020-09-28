/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_DFU_HOST_H__
#define UART_DFU_HOST_H__

/**
 * @file uart_dfu_host.h
 * @defgroup uart_dfu_host UART DFU host.
 * @{
 * @brief UART DFU host API.
 */

/**
 * @brief Initialize the UART DFU host.
 */
void uart_dfu_host_init(void);

/**
 * @brief Enable the UART DFU host to accept incoming firmware upgrades.
 */
void uart_dfu_host_enable(void);

/**
 * @brief Disable the UART DFU host, aborting any ongoing firmware upgrades.
 */
void uart_dfu_host_disable(void);

/** @} */

#endif /* UART_DFU_HOST_H__ */
