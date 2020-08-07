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


struct uart_dfu_host {
    struct uart_dfu_srv srv;
    size_t idx;
    atomic_t file_size;
    atomic_t initialized;
    u8_t fragment_buf[CONFIG_UART_DFU_HOST_MAX_FRAGMENT_SIZE];
};


int uart_dfu_host_init(struct uart_dfu_host *host, size_t idx);
int uart_dfu_host_enable(struct uart_dfu_host *host);
int uart_dfu_host_disable(struct uart_dfu_host *host);


#endif /* UART_DFU_HOST_H__ */
