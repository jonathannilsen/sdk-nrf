/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#include "sfts.h"

#include <zephyr/types.h>
#include <logging/log.h>

#include <bluetooth/gatt.h>
#include <bluetooth/uuid.h>
#include <bluetooth/conn.h>

LOG_MODULE_REGISTER(bt_gatt_sfts);

static const struct bt_gatt_stfs_cb *callbacks;

static struct stfs_conn_ctx {
	struct bt_conn *conn;
	bool transfer_active;
	u32_t bytes_remaining;
} conns[CONFIG_BT_MAX_CONN];

static int conn_ctx_index_get(struct bt_conn *conn)
{
	for (int i = 0; i < ARRAY_SIZE(conns); i++) {
		if (conns[i].conn == conn) {
			return i;
		}
	}
	return -1;
}

static struct stfs_conn_ctx *conn_ctx_get(struct bt_conn *conn)
{
	int i = conn_ctx_index_get(conn);
	if (i < 0) {
		return NULL;
	}
	return &conns[i];
}

static int conn_ctx_add(struct bt_conn *conn)
{
	/* Try to locate an unoccupied index. */
	int i = conn_ctx_index_get(NULL);
	if (i < 0) {
		return -ENOMEM;
	}

	conns[i].conn = bt_conn_ref(conn);
	return 0;
}

static int conn_ctx_clear(struct bt_conn *conn)
{
	struct stfs_conn_ctx *conn_ctx = conn_ctx_get(conn);
	if (!conn_ctx) {
		return -EINVAL;
	}

	if (conn_ctx->transfer_active)
	{
		/* Abort transfer without abort opcode. */
		callbacks->abort_cb(conn_ctx->conn);
	}

	bt_conn_unref(conn_ctx->conn);

	conn_ctx->conn = NULL;
	conn_ctx->transfer_active = false;
	conn_ctx->bytes_remaining = 0;

	return 0;
}

static ssize_t handle_op_new(struct stfs_conn_ctx *conn_ctx,
			     const u8_t *param, u16_t len)
{
	/* Validate operation. */
	if (len != 5) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_PDU);
	}
	if (conn_ctx->transfer_active) {
		return BT_GATT_ERR(BT_ATT_ERR_PROCEDURE_IN_PROGRESS);
	}

	u32_t file_size = *((u32_t *) param);
	if (!file_size) {
		return BT_GATT_ERR(BT_ATT_ERR_OUT_OF_RANGE);
	}

	/* Reply based on user callback. */
	switch (callbacks->new_cb(conn_ctx->conn, file_size)) {
	case 0:
		conn_ctx->bytes_remaining = file_size;
		conn_ctx->transfer_active = true;
		return len;
	case -ENOMEM:
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	case -ECANCELED:
		return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
	default:
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
}

static ssize_t handle_op_data(struct stfs_conn_ctx *conn_ctx,
			      const u8_t *param, u16_t len)
{
	u16_t param_len = len - 1;

	/* Validate operation. */
	if (!param_len) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_PDU);
	}
	if (!conn_ctx->transfer_active) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	if (conn_ctx->bytes_remaining < param_len) {
		return BT_GATT_ERR(BT_ATT_ERR_OUT_OF_RANGE);
	}

	/* Reply based on user callback. */
	switch (callbacks->data_cb(conn_ctx->conn, param, param_len)) {
	case 0:
		conn_ctx->bytes_remaining -= param_len;
		return len;
	case -ENOMEM:
		return BT_GATT_ERR(BT_ATT_ERR_INSUFFICIENT_RESOURCES);
	case -ECANCELED:
		return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
	default:
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
}

static ssize_t handle_op_complete(struct stfs_conn_ctx *conn_ctx,
				  const u8_t *param, u16_t len)
{
	/* Validate operation. */
	if (len != 5) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_PDU);
	}
	if (!conn_ctx->transfer_active) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}
	if (conn_ctx->bytes_remaining) {
		return BT_GATT_ERR(BT_ATT_ERR_OUT_OF_RANGE);
	}

	u32_t crc = *((u32_t *) param);

	/* Reply based on user callback. */
	switch (callbacks->complete_cb(conn_ctx->conn, crc)) {
	case 0:
		conn_ctx->transfer_active = false;
		return len;
	case -ECANCELED:
		return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
	default:
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
}

static ssize_t handle_op_abort(struct stfs_conn_ctx *conn_ctx,
			       const u8_t *param, u16_t len)
{
	/* Param should be empty. */
	if (len != 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_PDU);
	}
	if (!conn_ctx->transfer_active) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	callbacks->abort_cb(conn_ctx->conn);
	conn_ctx->transfer_active = false;
	conn_ctx->bytes_remaining = 0;

	return len;
}

static ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	const u8_t *data = buf;
	LOG_HEXDUMP_DBG(data, len, "RX: ");

	if (!callbacks) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
	if (!len) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_PDU);
	}

	struct stfs_conn_ctx *conn_ctx = conn_ctx_get(conn);
	if (!conn_ctx) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	const u8_t op = data[0];
	const u8_t *param = &data[1];

	switch (op) {
	case SFTS_OP_NEW:
		return handle_op_new(conn_ctx, param, len);
	case SFTS_OP_DATA:
		return handle_op_data(conn_ctx, param, len);
	case SFTS_OP_COMPLETE:
		return handle_op_complete(conn_ctx, param, len);
	case SFTS_OP_ABORT:
		return handle_op_abort(conn_ctx, param, len);
	default:
		return BT_GATT_ERR(BT_ATT_ERR_NOT_SUPPORTED);
	}
}

/* Simple File Transfer Service Declaration */
BT_GATT_SERVICE_DEFINE(sfts_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_SFTS_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_SFTS_DATA_CHAR, BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE, NULL, on_write, NULL),
);

static void on_connect(struct bt_conn *conn, u8_t err)
{
	LOG_INF("Connected.");

	int rc = conn_ctx_add(conn);
	if (rc) {
		LOG_ERR("Could not create connection context.");
	}
}

static void on_disconnect(struct bt_conn *conn, u8_t reason)
{
	LOG_INF("Disconnected.");

	int rc = conn_ctx_clear(conn);
	if (rc) {
		LOG_ERR("Could not clear connection context.");
	}
}

static struct bt_conn_cb conn_callbacks = {
	.connected = on_connect,
	.disconnected = on_disconnect,
};

int bt_gatt_stfs_init(const struct bt_gatt_stfs_cb *cb)
{
	if (!cb ||
	    !cb->new_cb || !cb->data_cb || !cb->complete_cb || !cb->abort_cb) {
		return -EINVAL;
	}

	callbacks = cb;
	bt_conn_cb_register(&conn_callbacks);

	return 0;
}
