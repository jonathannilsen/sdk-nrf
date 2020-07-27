/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <dfu/dfu_target.h>
#include <pm_config.h>
#include <logging/log.h>
#include <sys/printk.h>
#include <string.h>
#include <uart_dfu.h>
#include <sys/atomic.h>


/*****************************************************************************
 * Static variables 
 *****************************************************************************/

static const u8_t uart_header_magic[] = {
	0x85, 0xf3, 0xd8, 0x3a
};

static dfu_target_callback_t callback = NULL;

static struct uart_dfu_cli cli;
static atomic_t byte_counter = 0;
static atomic_t initialized = 0;
static int status_res = 0;
static size_t offset_res = 0;
K_SEM_DEFINE(dfu_target_uart_sem, 0, 1);

LOG_MODULE_REGISTER(dfu_target_uart, CONFIG_DFU_TARGET_LOG_LEVEL);


/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void cli_status_handle(int status, void * context)
{
	ARG_UNUSED(context);

	LOG_DBG("cli status(status=%d)", status);

	status_res = status;
	k_sem_give(&dfu_target_uart_sem);
}

static void cli_offset_handle(size_t offset, void * context)
{
	ARG_UNUSED(context);

	LOG_DBG("cli offset(offset=%u)", offset);
	
	status_res = 0;
	offset_res = offset;
	k_sem_give(&dfu_target_uart_sem);
}

static void cli_error_handle(int error, void * context)
{
	ARG_UNUSED(context);

	LOG_DBG("cli error(error=%d)", error);

	/* XXX: Differentiate between local and remote errors? */
	status_res = error;
	k_sem_give(&dfu_target_uart_sem);	
}

static int cli_init(void)
{
	int err;
	struct uart_dfu_cli_cb cli_cb;
	
	cli_cb.status_cb = cli_status_handle;
	cli_cb.offset_cb = cli_offset_handle;

	err = uart_dfu_cli_init(&cli, &cli_cb, cli_error_handle, NULL);
	if (err != 0) {
		return err;
	}
	return uart_dfu_cli_bind(CONFIG_DFU_TARGET_UART_INSTANCE_IDX, &cli);
}


/*****************************************************************************
 * API functions 
 *****************************************************************************/

bool dfu_target_uart_identify(const void * const buf)
{
	return memcmp(buf, uart_header_magic, sizeof(uart_header_magic)) == 0;
}

int dfu_target_uart_init(size_t file_size, dfu_target_callback_t cb)
{
	int err;

	/* TODO: ensure that this is not happening at a bad time. */
	k_sem_reset(&dfu_target_uart_sem);
	atomic_set(&byte_counter, 0);

	if (!atomic_set(&initialized, 1)) {
		err = cli_init();
		if (err != 0) {
			atomic_set(&initialized, 0);
			return err;	
		}
	}

	err = uart_dfu_cli_init_send(CONFIG_DFU_TARGET_UART_INSTANCE_IDX,
				     file_size);
	if (err != 0) {
		return err;
	}
	
	k_sem_take(&dfu_target_uart_sem, K_FOREVER);
	return status_res;
}

int dfu_target_uart_offset_get(size_t * offset)
{
	int err;

	err = uart_dfu_cli_offset_send(CONFIG_DFU_TARGET_UART_INSTANCE_IDX);
	if (err != 0) {
		return err;
	}
	k_sem_take(&dfu_target_uart_sem, K_FOREVER);
	if (status_res == 0) {
		*offset = offset_res;
	}
	return status_res;
}

int dfu_target_uart_write(const void * const buf, size_t len)
{
	int err;
	int count;
	u8_t * data;
	size_t data_len;

	/* Check that len will not cause an overflow of the counter. */
	if (len > INT32_MAX) {
		return -EINVAL;
	}

	count = atomic_add(&byte_counter, (atomic_t) len);
	if (count < sizeof(uart_header_magic)) {
		/* The magic number is stripped off the header
		   before transmission. */
		if (len > sizeof(uart_header_magic)) {
			data = &((u8_t *) buf)[sizeof(uart_header_magic)];
			data_len = len - sizeof(uart_header_magic);
		} else {
			/* The entirety of the buffer was stripped off;
			   return without transmitting. */
			return 0;
		}
	} else {
		data = (u8_t *) buf;
		data_len = len;
	}

	err = uart_dfu_cli_write_send(CONFIG_DFU_TARGET_UART_INSTANCE_IDX,
				      data,
				      data_len);
	if (err != 0) {
		return err;
	}

	k_sem_take(&dfu_target_uart_sem, K_FOREVER);
	return status_res;
}

int dfu_target_uart_done(bool successful)
{
	int err;

	/* TODO: better handling of cases where this is called at weird times.
			 e.g. call uart_dfu_cli_stop() if necessary. */

	err = uart_dfu_cli_done_send(CONFIG_DFU_TARGET_UART_INSTANCE_IDX,
				     successful);
	if (err != 0) {
		goto cleanup;
	}

	k_sem_take(&dfu_target_uart_sem, K_FOREVER);
	err = status_res;

cleanup:
	err = uart_dfu_cli_stop(CONFIG_DFU_TARGET_UART_INSTANCE_IDX);
	if (err == -EINPROGRESS) {
		k_sem_take(&dfu_target_uart_sem, K_FOREVER);
	}
	k_sem_reset(&dfu_target_uart_sem);
	atomic_set(&byte_counter, 0);
	return 0;
}