/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr.h>
#include <drivers/flash.h>
#include <storage/stream_flash_nv.h>
#include <logging/log.h>
#include <dfu/dfu_target.h>
#include <dfu/dfu_target_full_modem.h>

LOG_MODULE_REGISTER(dfu_target_full_modem, CONFIG_DFU_TARGET_LOG_LEVEL);

#define FULL_MODEM_HEADER_MAGIC 0x84d2

static struct stream_flash_nv_ctx stream;
#ifdef CONFIG_DFU_TARGET_SAVE_PROGRESS
static const char *stream_id = "DFU_FULL_MODEM";
#else
static const char *stream_id = NULL;
#endif
static bool configured;

bool dfu_target_full_modem_identify(const void *const buf)
{
	return *((const uint16_t *)buf) == FULL_MODEM_HEADER_MAGIC;
}

int dfu_target_full_modem_cfg(const struct dfu_target_full_modem_params *params)
{
	int err;

	err = stream_flash_nv_init(&stream, params->dev->dev, params->buf,
				   params->len, params->dev->offset,
				   params->dev->size, NULL, stream_id);
	if (err < 0) {
		LOG_ERR("stream_flash_nv_init failed %d", err);
		return err;
	}

	configured = true;

	return 0;
}

int dfu_target_full_modem_init(size_t file_size,
			       dfu_target_callback_t callback)
{
	if (!configured) {
		return -EPERM;
	}

	return 0;
}

int dfu_target_full_modem_offset_get(size_t *out)
{
	*out = stream_flash_nv_bytes_written(&stream);

	return 0;
}

int dfu_target_full_modem_write(const void *const buf, size_t len)
{
	int err = stream_flash_nv_buffered_write(&stream, buf, len, false);

	if (err != 0) {
		LOG_ERR("stream_flash_nv_buffered_write error %d", err);
		return err;
	}

	return err;
}

int dfu_target_full_modem_done(bool successful)
{
	int err = 0;

	configured = false;

	if (successful) {
		LOG_INF("Modem firmware downloaded to flash device");
		err = stream_flash_nv_buffered_write(&stream, NULL, 0, true);
		if (err != 0) {
			return err;
		}	
	}

	return stream_flash_nv_finish(&stream, successful);
}
