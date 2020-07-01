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
 * Macros
 *****************************************************************************/

#define UART_HEADER_MAGIC  		0x85f3d83a
#define UART_HEADER_MAGIC_SIZE 	4


/*****************************************************************************
 * Static variables 
 *****************************************************************************/

static struct uart_dfu_client dfu_client;
static dfu_target_callback_t callback = NULL;

static atomic_t byte_counter = 0;

static int result_status;
static size_t result_offset;
K_SEM_DEFINE(dfu_target_uart_sem, 0, 1);


/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void dfu_client_status_callback(int status)
{
	result_status = status;
	k_sem_give(&dfu_target_uart_sem);
}

static void dfu_client_offset_callback(size_t offset)
{
	result_status = 0;
	result_offset = offset;
	k_sem_give(&dfu_target_uart_sem);
}

static void dfu_client_available_callback(void)
{
	/* TODO */
}

static void dfu_client_error_callback(int error)
{
	/* XXX: Differentiate between local and remote errors? */
	result_status = error;
	k_sem_give(&dfu_target_uart_sem);	
}


/*****************************************************************************
 * API functions 
 *****************************************************************************/

bool dfu_target_uart_identify(const void * const buf)
{
	return *((const u32_t *) buf) == UART_HEADER_MAGIC;
}

int dfu_target_uart_init(size_t file_size, dfu_target_callback_t cb)
{
	int err_code;
	struct uart_dfu_client_callbacks dfu_client_callbacks;

	/* TODO: ensure that this is not happening at a bad time. */
	k_sem_reset(&dfu_target_uart_sem);
	atomic_set(&byte_counter, 0);

	dfu_client_callbacks.status_callback = dfu_client_status_callback;
	dfu_client_callbacks.offset_callback = dfu_client_offset_callback;
	dfu_client_callbacks.available_callback = dfu_client_available_callback;
	dfu_client_callbacks.error_callback = dfu_client_error_callback;

	err_code = uart_dfu_client_set(&dfu_client, &dfu_client_callbacks);
	if (err_code != 0)
	{
		/* TODO: ensure consistent error codes. */
		return err_code;
	}

	err_code = uart_dfu_client_init_send(file_size);
	if (err_code != 0)
	{
		/* TODO: cleanup first. */
		return err_code;
	}

	k_sem_take(&dfu_target_uart_sem, K_FOREVER);

	return result_status;
}

int dfu_target_uart_offset_get(size_t * offset)
{
	int err_code;

	err_code = uart_dfu_client_offset_send();
	if (err_code != 0)
	{
		return err_code;
	}

	k_sem_take(&dfu_target_uart_sem, K_FOREVER);

	if (result_status == 0)
	{
		*offset = result_offset;
	}

	return result_status;
}

int dfu_target_uart_write(const void * const buf, size_t len)
{
	int err_code;

	u8_t * data;
	size_t data_size;

	/* Check that len will not cause an overflow of the counter. */
	if (len > INT32_MAX)
	{
		return -EINVAL;
	}
	
	if (atomic_add(&byte_counter, (atomic_t) len) < UART_HEADER_MAGIC_SIZE)
	{
		/* The magic number is stripped off the header before transmission. */
		if (len > UART_HEADER_MAGIC_SIZE)
		{
			data = &((u8_t *) buf)[UART_HEADER_MAGIC_SIZE];
			data_size = len - UART_HEADER_MAGIC_SIZE;
		}
		else
		{
			/* The entirety of the buffer was stripped off; return without transmitting. */
			return 0;
		}
	}
	else
	{
		data = (u8_t *) buf;
		data_size = len;
	}

	err_code = uart_dfu_client_write_send(data, data_size);
	if (err_code != 0)
	{
		return err_code;
	}

	k_sem_take(&dfu_target_uart_sem, K_FOREVER);

	return result_status;
}

int dfu_target_uart_done(bool successful)
{
	int err_code;

	/* TODO: better handling of cases where this is called at weird times.
			 e.g. call uart_dfu_client_stop() if necessary. */

	err_code = uart_dfu_client_done_send(successful);
	if (err_code != 0)
	{
		goto cleanup;
	}

	k_sem_take(&dfu_target_uart_sem, K_FOREVER);
	err_code = result_status;

cleanup:
	if (uart_dfu_client_clear() != 0)
	{
		/* TODO: better handling. */
		k_oops();
	}

	return err_code;
}