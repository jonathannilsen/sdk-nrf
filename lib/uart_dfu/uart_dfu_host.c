/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <sys/atomic.h>
#include <uart_dfu.h>
#include <dfu/dfu_target.h>
#include <uart_dfu_host.h>


/* TODO: find a way to not do dfu_target calls from the uart_dfu
   workqueue context. */

/*****************************************************************************
* Static variables
*****************************************************************************/

LOG_MODULE_REGISTER(uart_dfu_host, CONFIG_UART_DFU_HOST_LOG_LEVEL);


/*****************************************************************************
* Static functions
*****************************************************************************/

static void target_evt_handle(enum dfu_target_evt_id evt_id)
{
	switch (evt_id) {
	case DFU_TARGET_EVT_TIMEOUT:
		/* TODO */
		break;
	case DFU_TARGET_EVT_ERASE_DONE:
		/* TODO */
		break;
	default:
		break;
	}
}

static int srv_init_handle(size_t file_size, void *context)
{
	struct uart_dfu_host *host = (struct uart_dfu_host *) context;

	if (atomic_cas(&host->file_size, 0, file_size)) {
		LOG_INF("Initialized(file_size=%u)", file_size);
		return 0;
	} else   {
		LOG_ERR("Initialize failed: busy.");
		return -EBUSY;
	}
}

static int srv_write_handle(const u8_t *const fragment_buf,
			    size_t fragment_size,
			    void *context)
{
	/* Init, write */
	int err;
	struct uart_dfu_host *host = (struct uart_dfu_host *) context;

	LOG_INF("Write(fragment_size=%u)", fragment_size);

	if (atomic_cas(&host->initialized, 0, 1)) {
		/* First fragment, initialize. */
		int img_type;

		img_type = dfu_target_img_type(fragment_buf, fragment_size);
		if (img_type < 0) {
			LOG_INF("dfu_target_img_type result: %d", img_type);
			return img_type;
		}

		err = dfu_target_init(img_type,
				      host->file_size,
				      target_evt_handle);
		if (err < 0) {
			LOG_INF("dfu_target_init result: %d", err);
			return err;
		}
	}

	err = dfu_target_write(fragment_buf, fragment_size);

	LOG_INF("dfu_target_write result: %d", err);

	return err;
}

static int srv_offset_handle(size_t *offset, void *context)
{
	int err;
	struct uart_dfu_host *host = (struct uart_dfu_host *) context;

	LOG_INF("Offset()");
	if (atomic_get(&host->initialized) == 0) {
		/* The target is not yet initialized,
		   so we just return an offset of 0. */
		*offset = 0;
		return 0;
	}

	err = dfu_target_offset_get(offset);
	/* XXX: Should we handle -EACCES here?
	    We may see it if dfu_target is used from another context. */
	return err;
}

static int srv_done_handle(bool successful, void *context)
{
	int err;
	struct uart_dfu_host *host = (struct uart_dfu_host *) context;

	LOG_INF("Done(successful=%u)", successful);

	err = dfu_target_done(successful);
	(void) atomic_set(&host->file_size, 0);
	(void) atomic_set(&host->initialized, 0);

	return err;
}

static void srv_error_handle(int error, void *context)
{
	LOG_ERR("Error(error=%d)", error);
	/* TODO: dfu_target_reset or similar? */
}


/*****************************************************************************
* API functions
*****************************************************************************/

int uart_dfu_host_init(struct uart_dfu_host *host, size_t idx)
{
	int err;
	struct uart_dfu_srv_cb callbacks;

	if (host == NULL) {
		return -EINVAL;
	}

	(void) atomic_set(&host->file_size, 0);
	(void) atomic_set(&host->initialized, 0);
	host->idx = idx;
	
	callbacks.init_cb = srv_init_handle;
	callbacks.write_cb = srv_write_handle;
	callbacks.offset_cb = srv_offset_handle;
	callbacks.done_cb = srv_done_handle;

	err = uart_dfu_srv_init(&host->srv,
				host->fragment_buf,
				CONFIG_UART_DFU_HOST_MAX_FRAGMENT_SIZE,
				&callbacks,
				srv_error_handle,
				host);
	if (err != 0) {
		return err;
	}
	err = uart_dfu_srv_bind(idx, &host->server);
	return err;
}

int uart_dfu_host_enable(struct uart_dfu_host *host)
{
	int err;

	if (host == NULL) {
		return -EINVAL;
	}

	err = uart_dfu_srv_enable(host->idx);

	return err;
}

int uart_dfu_host_disable(struct uart_dfu_host *host)
{
	int err;

	if (host == NULL) {
		return -EINVAL;
	}

	err = uart_dfu_srv_disable(host->idx);

	return err;
}
