/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef BT_GATT_SFTS_H_
#define BT_GATT_SFTS_H_

/**
 * @file
 * @defgroup bt_gatt_sfts BLE GATT Simple File Transfer Service API
 * @{
 * @brief API for the BLE GATT Simple File Transfer Service.
 */

#include <bluetooth/uuid.h>
#include <bluetooth/conn.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief UUID of the SFTS Service. **/
#define SFTS_UUID_SERVICE \
	BT_UUID_128_ENCODE(0xdb03a5c6, 0x86a8, 0x4c57, 0xb2ca, 0x367fbe53be0a)

/** @brief UUID of the SFTS Data Characteristic. **/
#define SFTS_UUID_DATA_CHAR \
	BT_UUID_128_ENCODE(0xdb03a5c7, 0x86a8, 0x4c57, 0xb2ca, 0x367fbe53be0a)

#define BT_UUID_SFTS_SERVICE	BT_UUID_DECLARE_128(SFTS_UUID_SERVICE)
#define BT_UUID_SFTS_DATA_CHAR	BT_UUID_DECLARE_128(SFTS_UUID_DATA_CHAR)

/** @brief Opcode for new file transfer request. */
#define SFTS_OP_NEW		0x00
/** @brief Opcode for data segment received. */
#define SFTS_OP_DATA		0x01
/** @brief Opcode for file transfer completed. */
#define SFTS_OP_COMPLETE	0x02
/** @brief Opcode for file transfer aborted by client. */
#define SFTS_OP_ABORT		0xFF

/** @brief Simple File Transfer Service event callbacks.
 *
 *  The application is responsible for validating and storing files
 *  transferred using this service. Therefore, all callbacks are mandatory.
 *
 *  @note Client misbehavior will be handled by the service implementation.
 *  The application can assume that all events for a particular connection
 *  will arrive in the correct order.
 */
struct bt_gatt_stfs_cb {
	/** @brief Callback for new file transfer request.
	 *
	 *  This callback is called whenever the service client at peer
	 *  wants to transfer a new file with a specific size. The server
	 *  gets an opportunity to check whether it has enough resources to
	 *  store the incoming file, and accept or reject the transfer.
	 *
	 *  @param conn		Pointer to associated connection object.
	 *  @param file_size	Size of the file to be transferred.
	 *
	 *  @retval 0		File accepted.
	 *  @retval -ENOMEM	File rejected because it is too big.
	 *  @retval -ECANCELED	File rejected for some other reason.
	 */
	int (*new_cb)(struct bt_conn *conn, const uint32_t file_size);

	/** @brief Callback for data segment received.
	 *
	 *  This callback is called whenever the service client at peer
	 *  sends a new part of the current file. The server has the option
	 *  to terminate the transfer after inspecting the file contents.
	 *  For example, the server may only allow files with a specific
	 *  header by accepting or rejecting the first data segment.
	 *
	 *  @param conn		Pointer to associated connection object.
	 *  @param data		Pointer to a data buffer.
	 *  @param len		Length of the data in the buffer.
	 *
	 *  @retval 0		Data segment accepted and stored.
	 *  @retval -ENOMEM	Not enough resources to complete transfer.
	 *  @retval -ECANCELED	Terminate unwanted transfer.
	 */
	int (*data_cb)(struct bt_conn *conn, const uint8_t *data, uint16_t len);

	/** @brief Callback for file transfer completed.
	 *
	 *  This callback is called whenever the service client at peer
	 *  signals that it has finished transmitting the current file.
	 *
	 *  @param conn		Pointer to associated connection object.
	 *  @param crc		CRC of the complete file for an optional check.
	 *
	 *  @retval 0		File transfer successful.
	 *  @retval -ECANCELED	File rejected, e.g. due to CRC error.
	 */
	int (*complete_cb)(struct bt_conn *conn, const uint32_t crc);

	/** @brief Callback for file transfer aborted by client.
	 *
	 *  This callback is called whenever the service client at peer
	 *  either sends an abort code or disconnects during a transfer.
	 *
	 *  @param conn		Pointer to associated connection object.
	 */
	void (*abort_cb)(struct bt_conn *conn);
};

/** @brief Initialize the service.
 *
 *  @param cb		Pointer to event callbacks.
 *
 *  @retval 0		Success.
 *  @retval -EINVAL	Null pointer error.
 */
int bt_gatt_stfs_init(const struct bt_gatt_stfs_cb *cb);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* BT_GATT_SFTS_H_ */
