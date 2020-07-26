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

static struct uart_dfu_client dfu_client;
static dfu_target_callback_t callback = NULL;

static atomic_t byte_counter = 0;

static int result_status;
static size_t result_offset;
K_SEM_DEFINE(dfu_target_uart_sem, 0, 1);

LOG_MODULE_REGISTER(dfu_target_uart, CONFIG_DFU_TARGET_LOG_LEVEL);


/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void dfu_client_status_callback(int status, void * context)
{
	ARG_UNUSED(context);

	LOG_DBG("dfu_client status(status=%d)", status);

	result_status = status;
	k_sem_give(&dfu_target_uart_sem);
}

static void dfu_client_offset_callback(size_t offset, void * context)
{
	ARG_UNUSED(context);

	LOG_DBG("dfu_client offset(offset=%u)", offset);
	
	result_status = 0;
	result_offset = offset;
	k_sem_give(&dfu_target_uart_sem);
}

static void dfu_client_error_callback(int error, void * context)
{
	ARG_UNUSED(context);

	LOG_DBG("dfu_client error(error=%d)", error);

	/* XXX: Differentiate between local and remote errors? */
	result_status = error;
	k_sem_give(&dfu_target_uart_sem);	
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
	struct uart_dfu_client_callbacks dfu_client_callbacks;

	/* TODO: ensure that this is not happening at a bad time. */
	k_sem_reset(&dfu_target_uart_sem);
	atomic_set(&byte_counter, 0);

	dfu_client_callbacks.status_callback = dfu_client_status_callback;
	dfu_client_callbacks.offset_callback = dfu_client_offset_callback;
	dfu_client_callbacks.error_callback = dfu_client_error_callback;

	err = uart_dfu_client_init(&dfu_client);
	if (err != 0)
	{
		return err;
	}

	err = uart_dfu_client_set(CONFIG_DFU_TARGET_UART_INSTANCE_IDX,
							  &dfu_client,
							  &dfu_client_callbacks,
							  NULL);
	if (err != 0)
	{
		return err;
	}

	err = uart_dfu_client_init_send(CONFIG_DFU_TARGET_UART_INSTANCE_IDX, file_size);
	if (err != 0)
	{
		/* TODO: cleanup first. */
		(void) uart_dfu_client_clear(CONFIG_DFU_TARGET_UART_INSTANCE_IDX);
		return err;
	}

	k_sem_take(&dfu_target_uart_sem, K_FOREVER);

	return result_status;
}

int dfu_target_uart_offset_get(size_t * offset)
{
	int err;

	err = uart_dfu_client_offset_send(CONFIG_DFU_TARGET_UART_INSTANCE_IDX);
	if (err != 0)
	{
		return err;
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
	int err;
	u8_t * data;
	size_t data_size;

	/* Check that len will not cause an overflow of the counter. */
	if (len > INT32_MAX)
	{
		return -EINVAL;
	}
	
	if (atomic_add(&byte_counter, (atomic_t) len) < sizeof(uart_header_magic))
	{
		/* The magic number is stripped off the header before transmission. */
		if (len > sizeof(uart_header_magic))
		{
			data = &((u8_t *) buf)[sizeof(uart_header_magic)];
			data_size = len - sizeof(uart_header_magic);
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

	err = uart_dfu_client_write_send(CONFIG_DFU_TARGET_UART_INSTANCE_IDX, data, data_size);
	if (err != 0)
	{
		return err;
	}

	k_sem_take(&dfu_target_uart_sem, K_FOREVER);

	return result_status;
}

int dfu_target_uart_done(bool successful)
{
	int err;

	/* TODO: better handling of cases where this is called at weird times.
			 e.g. call uart_dfu_client_stop() if necessary. */

	err = uart_dfu_client_done_send(CONFIG_DFU_TARGET_UART_INSTANCE_IDX, successful);
	if (err != 0)
	{
		goto cleanup;
	}

	k_sem_take(&dfu_target_uart_sem, K_FOREVER);
	err = result_status;

cleanup:
	(void) uart_dfu_client_clear(CONFIG_DFU_TARGET_UART_INSTANCE_IDX);
	return err;
}