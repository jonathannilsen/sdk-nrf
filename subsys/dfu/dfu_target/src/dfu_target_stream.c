/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr.h>
#include <logging/log.h>
#include <storage/stream_flash.h>
#include <stdio.h>
#include <dfu/dfu_target_stream.h>

#ifdef CONFIG_DFU_TARGET_STREAM_SAVE_PROGRESS
#define MODULE "dfu"
#endif /* CONFIG_DFU_TARGET_STREAM_SAVE_PROGRESS */

LOG_MODULE_REGISTER(dfu_target_stream, CONFIG_DFU_TARGET_LOG_LEVEL);

static struct stream_flash_ctx stream;
static const char *current_id;

#ifdef CONFIG_DFU_TARGET_STREAM_SAVE_PROGRESS

static char current_name_key[32];

#endif /* CONFIG_DFU_TARGET_STREAM_SAVE_PROGRESS */

struct stream_flash_ctx *dfu_target_stream_get_stream(void)
{
	return &stream;
}

int dfu_target_stream_init(const struct dfu_target_stream_init *init)
{
	int err;

	if (current_id != NULL) {
		return -EFAULT;
	}

	if (init == NULL || init->id == NULL || init->fdev == NULL ||
	    init->buf == NULL) {
		return -EINVAL;
	}

	current_id = init->id;

	err = stream_flash_init(&stream, init->fdev, init->buf, init->len,
				init->offset, init->size, NULL);
	if (err) {
		LOG_ERR("stream_flash_init failed (err %d)", err);
		return err;
	}

#ifdef CONFIG_DFU_TARGET_STREAM_SAVE_PROGRESS
	err = snprintf(current_name_key, sizeof(current_name_key), "%s/%s",
		       MODULE, current_id);
	if (err < 0 || err >= sizeof(current_name_key)) {
		LOG_ERR("Unable to generate current_name_key");
		return -EFAULT;
	}
	err = stream_flash_progress_enable(&stream, current_name_key, true);
	if (err) {
		LOG_ERR("settings_load failed (err %d)", err);
		return err;
	}
#endif /* CONFIG_DFU_TARGET_STREAM_SAVE_PROGRESS */

	return 0;
}

int dfu_target_stream_offset_get(size_t *out)
{
	*out = stream_flash_bytes_written(&stream);

	return 0;
}

int dfu_target_stream_write(const uint8_t *buf, size_t len)
{
	int err = stream_flash_buffered_write(&stream, buf, len, false);

	if (err != 0) {
		LOG_ERR("stream_flash_buffered_write error %d", err);
		return err;
	}

	return err;
}

int dfu_target_stream_done(bool successful)
{
	int err = 0;

	if (successful) {
		err = stream_flash_buffered_write(&stream, NULL, 0, true);
		if (err != 0) {
			LOG_ERR("stream_flash_buffered_write error %d", err);
		}
	}
#ifdef CONFIG_DFU_TARGET_STREAM_SAVE_PROGRESS
	/* If successful, delete progress so that a new call to 'init' will
	 * start with offset 0.
	 * Otherwise, retain progress so that a new call to 'init' will pick up
	 * where we left off.
	 */
	err = stream_flash_progress_disable(&stream, successful);
	if (err != 0) {
		LOG_ERR("stream_flash_progress_disable error %d", err);
	}
#endif
	
	current_id = NULL;

	return err;
}
