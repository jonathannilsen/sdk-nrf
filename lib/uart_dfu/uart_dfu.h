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
* Structure defintions 
*****************************************************************************/

/** UART DFU event types. */
enum uart_dfu_evt_type {
	/* TODO: update names of the below "session" events. */
	/** Session entered. */
	UART_DFU_EVT_SESS_ENTER,
	/** Session exited due to a user-specified error. */
	UART_DFU_EVT_SESS_EXIT,
	/** Received data. */
	UART_DFU_EVT_RX,
	/** Reception aborted by user or due to timeout or UART break. */
	UART_DFU_EVT_RX_END,
	/** Transmission completed or aborted by user or due to timeout. */
	UART_DFU_EVT_TX_END,
};

/** UART DFU event structure. */
struct uart_dfu_evt {
	enum uart_dfu_evt_type type;
	union {
		/** Event structure used by UART_DFU_EVT_RX. */
		struct {
			const uint8_t *buf;
			size_t len;
		} rx;
		/**
		 * Error value used by UART_DFU_EVT_SESS_EXIT,
		 * UART_DFU_EVT_RX_END, and UART_DFU_EVT_TX_END.
		 */
		int err;
	} data;
};

/** UART DFU event callback signature. */
typedef void (*uart_dfu_cb_t)(const struct uart_dfu_evt *const evt);

/** UART DFU session names. */
enum uart_dfu_sess {
	UART_DFU_SESS_IDLE,
	UART_DFU_SESS_SRV,
	UART_DFU_SESS_CLI,
	UART_DFU_SESS_SW,
	UART_DFU_SESS_COUNT
};


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

void uart_dfu_idle_set(uart_dfu_cb_t idle_cb);

int uart_dfu_user_request(uart_dfu_cb_t user_cb);

int uart_dfu_user_release(uart_dfu_cb_t user_cb, int err);

bool uart_dfu_in_work_q_thread(void);


#endif  /* UART_DFU_H__ */
