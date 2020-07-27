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


/*****************************************************************************
 * Macros
 *****************************************************************************/

#define UART_DFU_CLIENT_TX_BUF_SIZE		(255)	// Max COBS bytes 
#define UART_DFU_SERVER_TX_BUF_SIZE		(7)		// Max server message size


/*****************************************************************************
 * Structure definitions 
 *****************************************************************************/

struct uart_dfu_client_callbacks {
	void 	(*status_callback)(int status, void * context);
	void 	(*offset_callback)(size_t offset, void * context);
	
	/* TODO: improve when documenting.
	 * 
	 * -ECANCELED: Scheduled TX was unable to start
	 * -ENOTSUP: RX data invalid or not supported
	 * -EBUSY: RX could not start
	 * -EINVAL: Invalid value provided in callback
	 */
	void 	(*error_callback)(int error, void * context);
};

struct uart_dfu_client_protocol_state {
	atomic_t rx_opcode;
	atomic_t tx_state;
};

struct uart_dfu_client {
	const u8_t * fragment_buf;
	size_t 	fragment_idx;
	size_t  fragment_total_size;
	u8_t 	tx_data[UART_DFU_CLIENT_TX_BUF_SIZE];
	size_t 	tx_size;
	struct uart_dfu_client_protocol_state state;
};

struct uart_dfu_server_callbacks {
	int 	(*init_callback)(size_t file_size, void * context);
	int 	(*write_callback)(const u8_t * const fragment_buf,
						   	  size_t fragment_size,
							  void * context);
	int 	(*offset_callback)(size_t * offset, void * context);
	int 	(*done_callback)(bool successful, void * context);
	void 	(*error_callback)(int error, void * context);
};

struct uart_dfu_server_protocol_state {
	atomic_t rx_state;
	atomic_t tx_opcode;	
};

struct uart_dfu_server {
	u8_t * 	fragment_buf;
	size_t	fragment_max_size; 
	size_t 	fragment_idx;
	size_t  fragment_total_size;
	u8_t 	tx_data[UART_DFU_SERVER_TX_BUF_SIZE];
	size_t 	tx_size;
	struct uart_dfu_server_protocol_state state;
};


/*****************************************************************************
 * API functions 
 *****************************************************************************/

int uart_dfu_inst_idx_get(const char * uart_label);

int uart_dfu_init(size_t inst_idx);

int uart_dfu_uninit(size_t inst_idx);

int uart_dfu_client_init(struct uart_dfu_client * dfu_client);

int uart_dfu_client_set(size_t inst_idx,
						struct uart_dfu_client * dfu_client,
						struct uart_dfu_client_callbacks * callbacks,
						void * context);

int uart_dfu_client_clear(size_t inst_idx);

int uart_dfu_client_init_send(size_t inst_idx, size_t file_size);

int uart_dfu_client_write_send(size_t inst_idx,
							   const u8_t * const fragment_buf,
							   size_t fragment_size);

int uart_dfu_client_offset_send(size_t inst_idx);

int uart_dfu_client_done_send(size_t inst_idx, bool successful);

int uart_dfu_client_stop(size_t inst_idx);

int uart_dfu_server_init(struct uart_dfu_server * dfu_server,
						 u8_t * fragment_buf,
						 size_t fragment_max_size);

int uart_dfu_server_set(size_t inst_idx,
					    struct uart_dfu_server * server,
						struct uart_dfu_server_callbacks * callbacks,
						void * context);

int uart_dfu_server_clear(size_t inst_idx);

int uart_dfu_server_enable(size_t inst_idx);

int uart_dfu_server_disable(size_t inst_idx);

#endif  /* UART_DFU_H__ */
