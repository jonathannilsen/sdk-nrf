/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <logging/log.h>
#include <dfu/dfu_target.h>
#include <uart_blob_rx.h>
#include <uart_dfu_host.h>


static bool initialized;
static size_t total_len;
static uint8_t fragment[CONFIG_UART_DFU_HOST_MAX_FRAGMENT_SIZE];

static struct k_poll_signal sig_stop = K_POLL_SIGNAL_INITIALIZER(sig_stop);

LOG_MODULE_REGISTER(uart_dfu_host, CONFIG_UART_DFU_HOST_LOG_LEVEL);


static void state_reset(void)
{
	total_len = 0;
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

static int blob_init_handle(size_t blob_len)
{
	if (blob_len == 0) {
		return -EINVAL;
	}
	if (total_len == 0) {
		total_len = blob_len;
		LOG_INF("Initialized(file_size=%u)", blob_len);
		return 0;
	} else {
		LOG_ERR("Initialize failed: busy.");
		return -EBUSY;
	}
}

static int blob_offset_handle(size_t *offset)
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

static int blob_write_handle(const uint8_t *const fragment_buf,
			    size_t fragment_len)
{
	LOG_INF("Write(fragment_len=%u)", fragment_len);

	if (total_len == 0) {
		/* Write received without init first */
		return -EACCES;
	}
	if (!initialized) {
		/* First fragment, initialize. */
		int img_type;

		img_type = dfu_target_img_type(fragment_buf, fragment_len);
		if (img_type < 0) {
			return img_type;
		}

		int err = dfu_target_init(img_type, total_len,
					target_evt_handle);
		if (err < 0) {
			return err;
		}

		initialized = true;
	}
	return dfu_target_write(fragment_buf, fragment_len);
}

static int blob_done_handle(bool successful)
{
	LOG_INF("Done(successful=%u)", successful);

	int err = dfu_target_done(successful);
	if (err == 0 && successful) {
		state_reset();
	}
	return err;
}

static void blob_evt_handle(const struct uart_blob_evt *const evt)
{
	switch (evt->type) {
	case UART_BLOB_EVT_STARTED:
		k_poll_signal_reset(&sig_stop);
		break;
	case UART_BLOB_EVT_STOPPED:
		if (initialized) {
			(void) dfu_target_reset();
			state_reset();
		} else if (total_len != 0) {
			state_reset();
		}
		k_poll_signal_raise(&sig_stop, evt->status);
		break;
	default:
		break;
	}
}

static void blob_rx_stop_wait(void)
{
	struct k_poll_event evt = K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
							K_POLL_MODE_NOTIFY_ONLY,
                                    			&sig_stop);
	k_poll(&evt, 1, K_FOREVER);
	k_poll_signal_reset(&sig_stop);
}


void uart_dfu_host_init(void)
{
	struct uart_blob_rx_cb callbacks;

	state_reset();

	callbacks.init_cb	= blob_init_handle;
	callbacks.write_cb	= blob_write_handle;
	callbacks.offset_cb	= blob_offset_handle;
	callbacks.done_cb	= blob_done_handle;
	callbacks.evt_cb	= blob_evt_handle;

	(void) uart_blob_rx_init(fragment,
				 CONFIG_UART_DFU_HOST_MAX_FRAGMENT_SIZE,
				 &callbacks);
}

void uart_dfu_host_enable(void)
{
	uart_blob_rx_enable();
}

void uart_dfu_host_disable(void)
{
	int err = uart_blob_rx_disable();
	if (err == -EINPROGRESS) {
		/* Wait for stop */
		blob_rx_stop_wait();
	}
}
