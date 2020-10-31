/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_BLOB_TYPES_H__
#define UART_BLOB_TYPES_H__

#include <zephyr/types.h>

/**
 * @file uart_blob_types.h
 * @defgroup uart_blob_types UART BLOB PDU types.
 * @{
 * @brief Macros and structures used for the UART BLOB protocol.
 */

/** Opcode for INIT PDUs. */
#define UART_BLOB_OPCODE_INIT			0x00
/** Opcode for WRITEH PDUs. */
#define UART_BLOB_OPCODE_WRITEH			0x01
/** Opcode for WRITEC PDUs. */
#define UART_BLOB_OPCODE_WRITEC			0x02
/** Opcode for OFFSET PDUs. */
#define UART_BLOB_OPCODE_OFFSET			0x03
/** Opcode for DONE PDUs. */
#define UART_BLOB_OPCODE_DONE			0x04

/** UART BLOB PDU header structure. */
struct uart_blob_hdr {
	/** PDU opcode. */
	uint8_t opcode	: 3;
	/** PDU status bit. */
	uint8_t status	: 1;
	/** Reserved bits. */
	uint8_t rfu	: 4;
} __packed;

/** Argument structure for UART_BLOB_OPCODE_INIT. */
struct uart_blob_init_args {
	/** Size of transmission. */
	uint32_t blob_len;
} __packed;

/** Argument structure for UART_BLOB_OPCODE_WRITEH. */
struct uart_blob_writeh_args {
	/** Size of fragment. */
	uint32_t fragment_len;
} __packed;

/** Argument structure for UART_BLOB_OPCODE_WRITEC. */
struct uart_blob_writec_args {
	/** Variable length fragment data. */
	uint8_t data[0];
} __packed;

/** Argument structure for UART_BLOB_OPCODE_OFFSET. */
struct uart_blob_offset_args {
	/** Padding; not used */
	uint8_t padding[4];
} __packed;

/** Argument structure for UART_BLOB_OPCODE_DONE. */
struct uart_blob_done_args {
	/** Padding; not used */
	uint8_t padding1[3];
	/** Padding; not used */
	uint8_t padding2	: 7;
	/** Success bit. */
	uint8_t success 	: 1;
} __packed;

/** Argument structure for any opcode with status bit set. */
struct uart_blob_status_args {
	union {
		/** Offset value (positive). */
		uint32_t offset;
		/** Status value (negative). */
		int32_t status;
	} data;
} __packed;

/** UART BLOB PDU argument union. */
union uart_blob_args {
	struct uart_blob_status_args status;
	struct uart_blob_init_args init;
	struct uart_blob_offset_args offset;
	struct uart_blob_writeh_args writeh;
	struct uart_blob_writec_args writec;
	struct uart_blob_done_args done;
};

/** UART BLOB PDU structure. */
struct uart_blob_pdu {
	/** Header. */
	struct uart_blob_hdr hdr;
	/** Arguments. */
	union uart_blob_args args;
} __packed;

/** @} */

#endif  /* UART_BLOB_TYPES_H__ */
