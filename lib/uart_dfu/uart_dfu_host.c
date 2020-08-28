/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <uart_dfu.h>
#include <dfu/dfu_target.h>
#include <uart_dfu_host.h>
#include <uart_dfu_srv.h>


/*****************************************************************************
* Static variables
*****************************************************************************/

static bool initialized			= false;
static size_t total_size		= 0;
static uint8_t fragment[CONFIG_UART_DFU_HOST_MAX_FRAGMENT_SIZE];

static struct k_poll_signal sig_stop	= K_POLL_SIGNAL_INITIALIZER(sig_stop);

LOG_MODULE_REGISTER(uart_dfu_host, CONFIG_UART_DFU_HOST_LOG_LEVEL);


/*****************************************************************************
* Static functions
*****************************************************************************/

static void state_reset(void)
{
	total_size = 0;
	initialized = false;
}

static void target_evt_handle(enum dfu_target_evt_id evt_id)
{
	/* TODO: should these events be handled?  */
	switch (evt_id) {
	case DFU_TARGET_EVT_TIMEOUT:
		break;
	case DFU_TARGET_EVT_ERASE_DONE:
		break;
	default:
		break;
	}
}

static int srv_init_handle(size_t file_size)
{
	if (file_size == 0) {
		return -EINVAL;
	}
	if (total_size == 0) {
		total_size = file_size;
		LOG_INF("Initialized(file_size=%u)", file_size);
		return 0;
	} else {
		LOG_ERR("Initialize failed: busy.");
		return -EBUSY;
	}
}

static int srv_offset_handle(size_t *offset)
{
	LOG_INF("Offset()");

	if (!initialized) {
		/* The target is not yet initialized (first fragment
		   not received), so we just return an offset of 0. */
		*offset = 0;
		return 0;
	}

	return dfu_target_offset_get(offset);
}

static int srv_write_handle(const uint8_t *const fragment_buf,
			    size_t fragment_size)
{
	LOG_INF("Write(fragment_size=%u)", fragment_size);

	if (total_size == 0) {
		/* Write received without init first */
		return -EACCES;
	}
	if (!initialized) {
		/* First fragment, initialize. */
		int img_type;

		img_type = dfu_target_img_type(fragment_buf, fragment_size);
		if (img_type < 0) {
			return img_type;
		}

		int err = dfu_target_init(img_type, total_size,
					target_evt_handle);
		if (err < 0) {
			return err;
		}

		initialized = true;
	}
	return dfu_target_write(fragment_buf, fragment_size);
}

static int srv_done_handle(bool successful)
{
	LOG_INF("Done(successful=%u)", successful);

	int err = dfu_target_done(successful);
	if (err == 0 && successful) {
		state_reset();
	}
	return err;
}

static void srv_evt_handle(const struct uart_dfu_srv_evt *const evt)
{
	switch (evt->type) {
	case UART_DFU_SRV_EVT_STARTED:
		k_poll_signal_reset(&sig_stop);
		break;
	case UART_DFU_SRV_EVT_STOPPED:
		if (initialized) {
			(void) dfu_target_reset();
			state_reset();
		} else if (total_size != 0) {
			state_reset();
		}
		k_poll_signal_raise(&sig_stop, evt->err);
		break;
	default:
		break;
	}
}

static void srv_stop_wait(void)
{
	struct k_poll_event evt = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
							K_POLL_MODE_NOTIFY_ONLY,
                                    			&sig_stop);
	k_poll(&evt, 1, K_FOREVER);
	k_poll_signal_reset(&sig_stop);
}

/*****************************************************************************
* API functions
*****************************************************************************/

void uart_dfu_host_init(void)
{
	struct uart_dfu_srv_cb callbacks;

	state_reset();

	callbacks.init_cb	= srv_init_handle;
	callbacks.write_cb	= srv_write_handle;
	callbacks.offset_cb	= srv_offset_handle;
	callbacks.done_cb	= srv_done_handle;
	callbacks.evt_cb	= srv_evt_handle;

	(void) uart_dfu_srv_init(fragment,
				CONFIG_UART_DFU_HOST_MAX_FRAGMENT_SIZE,
				&callbacks);
}

void uart_dfu_host_enable(void)
{
	uart_dfu_srv_enable();
}

void uart_dfu_host_disable(void)
{
	int err = uart_dfu_srv_disable();
	if (err == -EINPROGRESS) {
		/* Wait for stop */
		srv_stop_wait();
	}
}
