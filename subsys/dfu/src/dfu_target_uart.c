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
#include <uart_blob_tx.h>
#include <sys/atomic.h>


/*****************************************************************************
 * Static variables
 *****************************************************************************/

static const uint8_t uart_header_magic[] = {
	0x85, 0xf3, 0xd8, 0x3a
};

static size_t byte_counter		= 0;
static bool initialized			= false;
static struct k_poll_signal sig_start	= K_POLL_SIGNAL_INITIALIZER(sig_start);
static struct k_poll_signal sig_stop	= K_POLL_SIGNAL_INITIALIZER(sig_stop);
static struct k_poll_signal sig_cb	= K_POLL_SIGNAL_INITIALIZER(sig_cb);
static size_t offset_res		= 0;

static struct k_poll_event events[] = {
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                    K_POLL_MODE_NOTIFY_ONLY,
                                    &sig_start, 0),
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                    K_POLL_MODE_NOTIFY_ONLY,
                                    &sig_stop, 0),
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SIGNAL,
                                    K_POLL_MODE_NOTIFY_ONLY,
                                    &sig_cb, 0),
};
static struct k_poll_event *sig_start_event = &events[0];
static struct k_poll_event *sig_stop_event = &events[1];
static struct k_poll_event *sig_cb_event = &events[2];

LOG_MODULE_REGISTER(dfu_target_uart, CONFIG_DFU_TARGET_LOG_LEVEL);


/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void blob_tx_status_handle(int status)
{
	k_poll_signal_raise(&sig_cb, status);
}

static void blob_tx_offset_handle(size_t offset)
{
	offset_res = offset;
	k_poll_signal_raise(&sig_cb, 0);
}

static void blob_tx_evt_handle(const struct uart_blob_tx_evt *const evt)
{
	switch (evt->type) {
	case UART_BLOB_TX_EVT_STARTED:
		k_poll_signal_raise(&sig_start, (int) evt->err);
		break;
	case UART_BLOB_TX_EVT_STOPPED:
		k_poll_signal_raise(&sig_stop, (int) evt->err);
		break;
	default:
		break;
	}
}

static void blob_tx_start_wait(void)
{
	k_poll(sig_start_event, 1, K_FOREVER);
	sig_start_event->state = K_POLL_STATE_NOT_READY;
	k_poll_signal_reset(&sig_start);
}

static void blob_tx_stop_wait(void)
{
	k_poll(sig_stop_event, 1, K_FOREVER);
	sig_stop_event->state = K_POLL_STATE_NOT_READY;
	k_poll_signal_reset(&sig_stop);
}

static int blob_tx_stop_cb_wait(int *status, size_t *offset)
{
	int result = 0;

	k_poll(sig_stop_event, 2, K_FOREVER);
	if (sig_stop_event->state == K_POLL_TYPE_SIGNAL) {
		result = sig_stop.result;
		k_poll_signal_reset(&sig_stop);
		sig_stop_event->state = K_POLL_STATE_NOT_READY;
	}
	if (sig_cb_event->state == K_POLL_TYPE_SIGNAL) {
		*status = sig_cb.result;
		if (offset != NULL) {
			*offset = offset_res;
		}
		k_poll_signal_reset(&sig_cb);
		sig_cb_event->state = K_POLL_STATE_NOT_READY;
	}

	return result;
}

static int blob_tx_status_wait(int *status)
{
	return blob_tx_stop_cb_wait(status, NULL);
}

static int blob_tx_offset_wait(int *status, size_t *offset)
{
	return blob_tx_stop_cb_wait(status, offset);
}

static int blob_tx_init(void)
{
	struct uart_blob_tx_cb blob_tx_cb = {
		.status_cb	= blob_tx_status_handle;
		.offset_cb	= blob_tx_offset_handle;
		.evt_cb		= blob_tx_evt_handle;
	};
	return uart_blob_tx_init(&blob_tx_cb);
}

static void update_state_reset(void)
{
	k_poll_signal_reset(&sig_start);
	k_poll_signal_reset(&sig_stop);
	k_poll_signal_reset(&sig_cb);
	offset_res	= 0;
	byte_counter	= 0;
}


/*****************************************************************************
 * API functions
 *****************************************************************************/

bool dfu_target_uart_identify(const void *const buf)
{
	return memcmp(buf, uart_header_magic, sizeof(uart_header_magic)) == 0;
}

int dfu_target_uart_init(size_t file_size, dfu_target_callback_t cb)
{
	int err;
	if (file_size < sizeof(uart_header_magic)) {
		return -EINVAL;
	}

	update_state_reset();
	if (!initialized) {
		err = blob_tx_init();
		if (err != 0) {
			return err;
		}
		initialized = true;
	}

	err = uart_blob_tx_start();
	if (err == -EINPROGRESS) {
		/* Wait for start */
		LOG_DBG("Waiting for client start.");
		blob_tx_start_wait();
	} else if (err != 0) {
		LOG_ERR("Unable to start client: %d", err);
		return err;
	}

	err = uart_blob_tx_send_init(file_size - sizeof(uart_header_magic));
	if (err != 0) {
		LOG_ERR("Error sending init: %d", err);
		return err;
	}

	/* Wait for reply. */
	int status;
	err = blob_tx_status_wait(&status);
	if (err == 0) {
		LOG_DBG("Init sent. Received: %d.", status);
		return status;
	} else {
		LOG_ERR("Stopped with err %d while sending init", err);
		return err;
	}
}

int dfu_target_uart_offset_get(size_t *offset)
{
	if (offset == NULL) {
		return -EINVAL;
	}

	int err = uart_blob_tx_send_offset();
	if (err != 0) {
		LOG_ERR("Error sending offset: %d", err);
		return err;
	}

	/* Wait for reply. */
	int status;
	err = blob_tx_offset_wait(&status, offset);
	if (err == 0) {
		LOG_DBG("Offset sent. Received status: %d, offset: %u.",
			status, *offset);
		return status;
	} else {
		LOG_ERR("Stopped with err %d while sending offset", err);
		return err;
	}
}

int dfu_target_uart_write(const void *const buf, size_t len)
{
	uint8_t * data;
	size_t data_len;

	/* Check that len will not cause an overflow of the counter. */
	if (buf == NULL || len > INT32_MAX) {
		return -EINVAL;
	}

	if (byte_counter < sizeof(uart_header_magic)) {
		/* The magic number is stripped off the header
		   before transmission. */
		if (len > sizeof(uart_header_magic)) {
			data = &((uint8_t *) buf)[sizeof(uart_header_magic)];
			data_len = len - sizeof(uart_header_magic);
		} else {
			/* The entirety of the buffer was stripped off;
			   return without transmitting. */
			byte_counter = len;
			return 0;
		}
	} else {
		data = (uint8_t *) buf;
		data_len = len;
	}

	int err = uart_blob_tx_send_write(data, data_len);
	if (err != 0) {
		LOG_ERR("Error sending write: %d", err);
		return err;
	}
	byte_counter += len;

	/* Wait for reply. */
	int status;
	err = blob_tx_status_wait(&status);
	if (err == 0) {
		LOG_DBG("Write sent. Received: %d.", status);
		return status;
	} else {
		LOG_ERR("Stopped with err %d while sending write", err);
		return err;
	}
}

int dfu_target_uart_done(bool successful)
{
	int status;
	int err = uart_blob_tx_send_done(successful);
	if (err == 0) {
		/* Wait for reply. */
		err = blob_tx_status_wait(&status);
		if (err == 0) {
			LOG_DBG("Done sent. Received: %d.", status);
		} else {
			LOG_ERR("Stopped with err %d while sending done", err);
			status = err;
		}
	} else {
		LOG_ERR("Error sending done: %d", err);
		status = err;
	}

	err = uart_blob_tx_stop();
	if (err == -EINPROGRESS) {
		/* Wait for stop. */
		LOG_DBG("Waiting for client stop.");
		blob_tx_stop_wait();
	} else if (err != 0) {
		LOG_ERR("Unable to stop client: %d", err);
	}

	update_state_reset();
	return status;
}
