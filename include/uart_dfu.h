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

#define UART_DFU_CLI_TX_BUF_SIZE        (255)   // Max COBS bytes
#define UART_DFU_SRV_TX_BUF_SIZE        (7)     // Max server message size


/*****************************************************************************
* Structure definitions
*****************************************************************************/

struct uart_dfu_buf_in {
	const u8_t *buf;
	size_t idx;
	size_t size;
};

struct uart_dfu_buf_out {
	u8_t *buf;
	size_t idx;
	size_t size;
	size_t max_size;
};

/* TODO: figure out where this should be placed. */
/* TODO: improve when documenting.
 *
 * -ECANCELED: Scheduled RX/TX canceled 
 * -ETIMEDOUT: RX/TX timeout
 * -ENOTSUP: RX data invalid or not supported
 * -EINVAL: Invalid value provided in callback
 */
typedef void (*uart_dfu_error_t)(int error, void *context);

struct uart_dfu_inst {
	atomic_t flags;
	atomic_t tx_state;
	atomic_t rx_state;
	u8_t *tx_buf;
	size_t tx_size;
	size_t tx_buf_size;
	struct k_timer timer;
	atomic_t timeout;
	uart_dfu_error_t error_cb;
	void *mod_ctx;
};

struct uart_dfu_cli_cb {
	void (*status_cb)(int status, void *context);
	void (*offset_cb)(size_t offset, void *context);
};

struct uart_dfu_cli {
	struct uart_dfu_inst inst;
	struct uart_dfu_buf_in fragment;
	u8_t buf[UART_DFU_CLI_TX_BUF_SIZE];
	struct uart_dfu_cli_cb cb;
	void *ctx;
};

struct uart_dfu_srv_cb {
	int (*init_cb)(size_t file_size, void *context);
	int (*write_cb)(const u8_t *const fragment_buf,
			size_t fragment_size,
			void *context);
	int (*offset_cb)(size_t *offset, void *context);
	int (*done_cb)(bool successful, void *context);
};

struct uart_dfu_srv {
	struct uart_dfu_inst inst;
	struct uart_dfu_buf_out fragment;
	u8_t buf[UART_DFU_SRV_TX_BUF_SIZE];
	struct uart_dfu_srv_cb cb;
	void *ctx;
};


/*****************************************************************************
* API functions
*****************************************************************************/

int uart_dfu_idx_get(const char *uart_label);

int uart_dfu_init(size_t idx);

int uart_dfu_uninit(size_t idx);

int uart_dfu_cli_init(struct uart_dfu_cli *client,
		      struct uart_dfu_cli_cb *callbacks,
		      uart_dfu_error_t error_cb,
		      void *context);

int uart_dfu_cli_bind(size_t idx, struct uart_dfu_cli *client);

int uart_dfu_cli_unbind(size_t idx);

int uart_dfu_cli_init_send(size_t idx, size_t file_size);

int uart_dfu_cli_write_send(size_t idx,
			    const u8_t *const fragment_buf,
			    size_t fragment_size);

int uart_dfu_cli_offset_send(size_t idx);

int uart_dfu_cli_done_send(size_t idx, bool successful);

int uart_dfu_cli_stop(size_t idx);

int uart_dfu_srv_init(struct uart_dfu_srv *server,
		      u8_t *fragment_buf,
		      size_t fragment_max_size,
		      struct uart_dfu_srv_cb *callbacks,
		      uart_dfu_error_t error_cb,
		      void *context);

int uart_dfu_srv_bind(size_t idx, struct uart_dfu_srv *server);

int uart_dfu_srv_unbind(size_t idx);

int uart_dfu_srv_enable(size_t idx);

int uart_dfu_srv_disable(size_t idx);


#endif  /* UART_DFU_H__ */
