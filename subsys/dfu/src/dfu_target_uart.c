/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <dfu/dfu_target.h>
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
static size_t byte_counter = 0;
static bool initialized = false;
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
	status_res = status;
	k_sem_give(&dfu_target_uart_sem);
}

static void cli_offset_handle(size_t offset, void * context)
{
	ARG_UNUSED(context);
	status_res = 0;
	offset_res = offset;
	k_sem_give(&dfu_target_uart_sem);
}

static void cli_error_handle(int error, void * context)
{
	ARG_UNUSED(context);
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
	return uart_dfu_cli_bind(CONFIG_DFU_TARGET_UART_INSTANCE, &cli);
}

static void session_state_reset(void)
{
	k_sem_reset(&dfu_target_uart_sem);
	byte_counter = 0;
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

	if (file_size < sizeof(uart_header_magic)) {
		return -EINVAL;
	}

	session_state_reset();
	if (!initialized) {
		err = cli_init();
		if (err != 0) {
			return err;	
		}
		initialized = true;
	}

	err = uart_dfu_cli_init_send(CONFIG_DFU_TARGET_UART_INSTANCE,
				     file_size - sizeof(uart_header_magic));
	if (err != 0) {
		return err;
	}
	k_sem_take(&dfu_target_uart_sem, K_FOREVER);
	return status_res;
}

int dfu_target_uart_offset_get(size_t * offset)
{
	int err;

	if (offset == NULL) {
		return -EINVAL;
	}

	err = uart_dfu_cli_offset_send(CONFIG_DFU_TARGET_UART_INSTANCE);
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
	u8_t * data;
	size_t data_len;

	/* Check that len will not cause an overflow of the counter. */
	if (buf == NULL || len > INT32_MAX) {
		return -EINVAL;
	}

	if (byte_counter < sizeof(uart_header_magic)) {
		/* The magic number is stripped off the header
		   before transmission. */
		if (len > sizeof(uart_header_magic)) {
			data = &((u8_t *) buf)[sizeof(uart_header_magic)];
			data_len = len - sizeof(uart_header_magic);
		} else {
			/* The entirety of the buffer was stripped off;
			   return without transmitting. */
			byte_counter = len;
			return 0;
		}
	} else {
		data = (u8_t *) buf;
		data_len = len;
	}

	err = uart_dfu_cli_write_send(CONFIG_DFU_TARGET_UART_INSTANCE,
				      data,
				      data_len);
	if (err != 0) {
		return err;
	}
	byte_counter += len;
	
	k_sem_take(&dfu_target_uart_sem, K_FOREVER);
	return status_res;
}

int dfu_target_uart_done(bool successful)
{
	int err;

	err = uart_dfu_cli_stop(CONFIG_DFU_TARGET_UART_INSTANCE);
	if (err == -EINPROGRESS) {
		k_sem_take(&dfu_target_uart_sem, K_FOREVER);
	}
	
	err = uart_dfu_cli_done_send(CONFIG_DFU_TARGET_UART_INSTANCE,
				     successful);
	if (err == 0) {
		k_sem_take(&dfu_target_uart_sem, K_FOREVER);
		err = status_res;
	}

	session_state_reset();
	return err;
}
