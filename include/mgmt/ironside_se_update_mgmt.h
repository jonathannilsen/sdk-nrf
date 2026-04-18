/*
 * Copyright (c) 2026 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef IRONSIDE_SE_UPDATE_MGMT_H__
#define IRONSIDE_SE_UPDATE_MGMT_H__

/**
 * @file ironside_se_update_mgmt.h
 * @defgroup ironside_se_update_mgmt MCUmgr group for IronSide SE firmware update
 * @{
 * @brief MCUmgr-based IronSide SE firmware update over SMP.
 *
 * Registers a custom MCUmgr command group that accepts chunked firmware
 * uploads into a staging flash partition and calls ironside_se_update()
 * once the full update blob has been received.
 *
 * The group auto-registers during system init when
 * CONFIG_MGMT_IRONSIDE_SE_UPDATE is enabled.  A DTS partition labeled
 * ``ironside_se_update_partition`` must exist within the valid IronSide SE
 * update address range.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * MCUmgr group ID for IronSide SE update.
 * Uses the per-user range so it does not collide with standard groups.
 */
#define MGMT_GROUP_ID_IRONSIDE_SE_UPDATE (MGMT_GROUP_ID_PERUSER + 2)

/** Command IDs within the IronSide SE update group. */
#define IRONSIDE_SE_UPDATE_MGMT_ID_UPLOAD      0
#define IRONSIDE_SE_UPDATE_MGMT_ID_VERSION_GET 1
#define IRONSIDE_SE_UPDATE_MGMT_ID_STATUS_GET  2

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* IRONSIDE_SE_UPDATE_MGMT_H__ */
