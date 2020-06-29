/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_DFU_H__
#define UART_DFU_H__

#include <stddef.h>
#include <zephyr/types.h>
#include <sys/atomic.h>
#include <uart_dfu_types.h>


/*****************************************************************************
* API functions
*****************************************************************************/

uint8_t *uart_dfu_tx_buf_get(void);

int uart_dfu_tx_start(size_t len, int timeout);

int uart_dfu_tx_stop(void);

int uart_dfu_rx_start(size_t len);

int uart_dfu_rx_stop(void);

void uart_dfu_rx_timeout_start(int timeout);

void uart_dfu_rx_timeout_stop(void);

int uart_dfu_sess_open(enum uart_dfu_sess sess);

int uart_dfu_sess_close(enum uart_dfu_sess sess, int err);

bool uart_dfu_in_work_q_thread(void);


#endif  /* UART_DFU_H__ */
