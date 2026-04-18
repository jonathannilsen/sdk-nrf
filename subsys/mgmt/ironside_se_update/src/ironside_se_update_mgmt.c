/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/mgmt/handlers.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/storage/stream_flash.h>
#include <ironside/se/api.h>
#include <ironside/se/boot_report.h>
#include <ironside/se/memory_map.h>
#include <mgmt/ironside_se_update_mgmt.h>
#include <zcbor_common.h>
#include <zcbor_decode.h>
#include <zcbor_encode.h>
#include <mgmt/mcumgr/util/zcbor_bulk.h>

LOG_MODULE_REGISTER(ironside_se_update_mgmt, CONFIG_MGMT_IRONSIDE_SE_UPDATE_LOG_LEVEL);

#define STAGING_LABEL	ironside_se_update_partition
#define STAGING_ID	FIXED_PARTITION_ID(STAGING_LABEL)
#define STAGING_SIZE	FIXED_PARTITION_SIZE(STAGING_LABEL)
#define STAGING_OFFSET	FIXED_PARTITION_OFFSET(STAGING_LABEL)
#define STAGING_ADDRESS FIXED_PARTITION_ADDRESS(STAGING_LABEL)

BUILD_ASSERT(STAGING_ADDRESS >= IRONSIDE_SE_UPDATE_MIN_ADDRESS &&
		     STAGING_ADDRESS <= IRONSIDE_SE_UPDATE_MAX_ADDRESS,
	     "Staging partition must be within the valid IronSide SE update address range");
BUILD_ASSERT(STAGING_SIZE >= IRONSIDE_SE_UPDATE_BLOB_SIZE,
	     "Staging partition must be >= IRONSIDE_SE_UPDATE_BLOB_SIZE (160 KiB)");

static struct stream_flash_ctx stream;
static uint8_t stream_buf[CONFIG_MGMT_IRONSIDE_SE_UPDATE_WRITE_BUF_SIZE];
static uint32_t upload_total_len;
static uint32_t upload_bytes_written;
static bool upload_complete;

static int encode_response(struct smp_streamer *ctxt, uint32_t offset)
{
	zcbor_state_t *zse = ctxt->writer->zs;
	bool ok;

	ok = zcbor_tstr_put_lit(zse, "rc") && zcbor_int32_put(zse, MGMT_ERR_EOK) &&
	     zcbor_tstr_put_lit(zse, "off") && zcbor_uint32_put(zse, offset);

	return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

static int ironside_se_upload(struct smp_streamer *ctxt)
{
	zcbor_state_t *zsd = ctxt->reader->zs;
	uint64_t offset = 0;
	uint64_t file_len = 0;
	struct zcbor_string data = {0};
	size_t decoded = 0;
	const struct flash_area *fa;
	int rc;

	struct zcbor_map_decode_key_val upload_decode[] = {
		ZCBOR_MAP_DECODE_KEY_VAL(off, zcbor_uint64_decode, &offset),
		ZCBOR_MAP_DECODE_KEY_VAL(data, zcbor_bstr_decode, &data),
		ZCBOR_MAP_DECODE_KEY_VAL(len, zcbor_uint64_decode, &file_len),
	};

	if (zcbor_map_decode_bulk(zsd, upload_decode, ARRAY_SIZE(upload_decode), &decoded) != 0) {
		return MGMT_ERR_EINVAL;
	}

	rc = flash_area_open(STAGING_ID, &fa);
	if (rc) {
		LOG_ERR("Failed to open staging partition: %d", rc);
		return MGMT_ERR_EUNKNOWN;
	}

	if (offset == 0) {
		if (file_len == 0 || file_len > STAGING_SIZE) {
			LOG_ERR("Invalid blob length: %llu (max %u)", (unsigned long long)file_len,
				STAGING_SIZE);
			flash_area_close(fa);
			return MGMT_ERR_EINVAL;
		}
		upload_total_len = (uint32_t)file_len;
		upload_bytes_written = 0;
		upload_complete = false;

		rc = flash_area_erase(fa, 0, upload_total_len);
		if (rc) {
			LOG_ERR("Staging area erase failed: %d", rc);
			flash_area_close(fa);
			return MGMT_ERR_EUNKNOWN;
		}

		rc = stream_flash_init(&stream, flash_area_get_device(fa), stream_buf,
				       sizeof(stream_buf), fa->fa_off, upload_total_len, NULL);
		if (rc) {
			LOG_ERR("stream_flash_init failed: %d", rc);
			flash_area_close(fa);
			return MGMT_ERR_EUNKNOWN;
		}

		LOG_INF("IronSide SE update: upload started (%u bytes)", upload_total_len);
	}

	if ((uint32_t)offset != upload_bytes_written) {
		LOG_WRN("Unexpected offset %u (expected %u), responding with current position",
			(uint32_t)offset, upload_bytes_written);
	} else if (data.len > 0 && !upload_complete) {
		bool last = (upload_bytes_written + data.len) >= upload_total_len;

		rc = stream_flash_buffered_write(&stream, data.value, data.len, last);
		if (rc) {
			LOG_ERR("Flash write failed at offset %u: %d", upload_bytes_written, rc);
			flash_area_close(fa);
			return MGMT_ERR_EUNKNOWN;
		}
		upload_bytes_written += (uint32_t)data.len;

		if (upload_bytes_written >= upload_total_len) {
			LOG_INF("Upload complete, requesting IronSide SE update");

			const struct ironside_se_update_blob *blob =
				(const struct ironside_se_update_blob *)STAGING_ADDRESS;

			rc = ironside_se_update(blob);
			if (rc) {
				LOG_ERR("ironside_se_update() failed: %d", rc);
				flash_area_close(fa);
				return MGMT_ERR_EUNKNOWN;
			}
			upload_complete = true;
			LOG_INF("Update requested — reboot to apply");
		}
	}

	flash_area_close(fa);

	return encode_response(ctxt, upload_bytes_written);
}

static int encode_slot_version(zcbor_state_t *zse, const char *key, uint32_t version_int,
			       const char *extraversion)
{
	bool ok;

	ok = zcbor_tstr_put_term(zse, key, CONFIG_MCUMGR_TRANSPORT_NETBUF_SIZE) &&
	     zcbor_map_start_encode(zse, 2) && zcbor_tstr_put_lit(zse, "version_int") &&
	     zcbor_uint32_put(zse, version_int) && zcbor_tstr_put_lit(zse, "extraversion") &&
	     zcbor_tstr_put_term(zse, extraversion, CONFIG_MCUMGR_TRANSPORT_NETBUF_SIZE) &&
	     zcbor_map_end_encode(zse, 2);

	return ok ? 0 : MGMT_ERR_EMSGSIZE;
}

static int ironside_se_version_get(struct smp_streamer *ctxt)
{
	const struct ironside_se_boot_report *report = IRONSIDE_SE_BOOT_REPORT;
	zcbor_state_t *zse = ctxt->writer->zs;
	int rc;

	if (report->magic != IRONSIDE_SE_BOOT_REPORT_MAGIC) {
		LOG_ERR("Boot report not available (bad magic)");
		return MGMT_ERR_ENOENT;
	}

	rc = encode_slot_version(zse, "uslot", report->ironside_se_version_int,
				 report->ironside_se_extraversion);
	if (rc) {
		return rc;
	}

	rc = encode_slot_version(zse, "rslot", report->ironside_se_recovery_version_int,
				 report->ironside_se_recovery_extraversion);

	return rc;
}

static int ironside_se_status_get(struct smp_streamer *ctxt)
{
	const struct ironside_se_boot_report *report = IRONSIDE_SE_BOOT_REPORT;
	zcbor_state_t *zse = ctxt->writer->zs;
	bool ok;

	if (report->magic != IRONSIDE_SE_BOOT_REPORT_MAGIC) {
		LOG_ERR("Boot report not available (bad magic)");
		return MGMT_ERR_ENOENT;
	}

	ok = zcbor_tstr_put_lit(zse, "status") &&
	     zcbor_uint32_put(zse, report->ironside_update_status);

	return ok ? MGMT_ERR_EOK : MGMT_ERR_EMSGSIZE;
}

static const struct mgmt_handler ironside_se_update_handlers[] = {
	[IRONSIDE_SE_UPDATE_MGMT_ID_UPLOAD] =
		{
			.mh_read = NULL,
			.mh_write = ironside_se_upload,
		},
	[IRONSIDE_SE_UPDATE_MGMT_ID_VERSION_GET] =
		{
			.mh_read = ironside_se_version_get,
			.mh_write = NULL,
		},
	[IRONSIDE_SE_UPDATE_MGMT_ID_STATUS_GET] =
		{
			.mh_read = ironside_se_status_get,
			.mh_write = NULL,
		},
};

static struct mgmt_group ironside_se_update_grp = {
	.mg_handlers = ironside_se_update_handlers,
	.mg_handlers_count = ARRAY_SIZE(ironside_se_update_handlers),
	.mg_group_id = MGMT_GROUP_ID_IRONSIDE_SE_UPDATE,
};

static void ironside_se_update_mgmt_register(void)
{
	mgmt_register_group(&ironside_se_update_grp);
}

MCUMGR_HANDLER_DEFINE(ironside_se_update_mgmt, ironside_se_update_mgmt_register);
