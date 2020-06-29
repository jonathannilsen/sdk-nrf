/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_DFU_TYPES_H__
#define UART_DFU_TYPES_H__

#include <zephyr/types.h>

/**
 * @file uart_dfu_types.h
 * @defgroup uart_dfu_types UART DFU PDU types.
 * @{
 * @brief Macros and structures used for the UART DFU protocol.
 */

/*****************************************************************************
 * Macros
 *****************************************************************************/

/** Opcode for INIT PDUs. */
#define UART_DFU_OPCODE_INIT			0x00
/** Opcode for WRITEH PDUs. */
#define UART_DFU_OPCODE_WRITEH			0x01
/** Opcode for WRITEC PDUs. */
#define UART_DFU_OPCODE_WRITEC			0x02
/** Opcode for OFFSET PDUs. */
#define UART_DFU_OPCODE_OFFSET			0x03
/** Opcode for DONE PDUs. */
#define UART_DFU_OPCODE_DONE			0x04


/*****************************************************************************
 * Structure definitions
 *****************************************************************************/

/** UART DFU PDU header structure. */
struct uart_dfu_hdr {
	/** PDU opcode. */
	uint8_t opcode	: 3;
	/** PDU status bit. */
	uint8_t status	: 1;
	/** Reserved bits. */
	uint8_t rfu	: 4;
} __packed;

/** Argument structure for UART_DFU_OPCODE_INIT. */
struct uart_dfu_init_args {
	/** Size of transmission. */
	uint32_t file_size;
} __packed;

/** Argument structure for UART_DFU_OPCODE_WRITEH. */
struct uart_dfu_writeh_args {
	/** Size of fragment. */
	uint32_t fragment_size;
} __packed;

/** Argument structure for UART_DFU_OPCODE_WRITEC. */
struct uart_dfu_writec_args {
	/** Variable length fragment data. */
	uint8_t data[0];
} __packed;

/** Argument structure for UART_DFU_OPCODE_OFFSET. */
struct uart_dfu_offset_args {
	/** Padding; not used */
	uint8_t padding[4];
} __packed;

/** Argument structure for UART_DFU_OPCODE_DONE. */
struct uart_dfu_done_args {
	/** Padding; not used */
	uint8_t padding1[3];
	/** Padding; not used */
	uint8_t padding2	: 7;
	/** Success bit. */
	uint8_t success 	: 1;
} __packed;

/** Argument structure for any opcode with status bit set. */
struct uart_dfu_status_args {
	union {
		/** Offset value (positive). */
		uint32_t offset;
		/** Status value (negative). */
		int32_t status;
	} data;
} __packed;

/** UART DFU PDU argument union. */
union uart_dfu_args {
	struct uart_dfu_status_args status;
	struct uart_dfu_init_args init;
	struct uart_dfu_offset_args offset;
	struct uart_dfu_writeh_args writeh;
	struct uart_dfu_writec_args writec;
	struct uart_dfu_done_args done;
};

/** UART DFU PDU structure. */
struct uart_dfu_pdu {
	/** Header. */
	struct uart_dfu_hdr hdr;
	/** Arguments. */
	union uart_dfu_args args;
} __packed;

/** UART DFU event types. */
enum uart_dfu_evt_type {
	/** Session entered. */
	UART_DFU_EVT_SESS_ENTER,
	/** Session exited due to a user-specified error. */
	UART_DFU_EVT_SESS_EXIT,
	/** Received data. */
	UART_DFU_EVT_RX,
	/** Reception aborted by user or due to timeout or UART break. */
	UART_DFU_EVT_RX_END,
	/** Transmission completed or aborted by user or due to timeout. */
	UART_DFU_EVT_TX_END,
};

/** UART DFU event structure. */
struct uart_dfu_evt {
	enum uart_dfu_evt_type type;
	union {
		/** Event structure used by UART_DFU_EVT_RX. */
		struct {
			struct uart_dfu_pdu *pdu;
			size_t len;
		} rx;
		/**
		 * Error value used by UART_DFU_EVT_SESS_EXIT,
		 * UART_DFU_EVT_RX_END, and UART_DFU_EVT_TX_END.
		 */
		int err;
	} data;
};

/** UART DFU event callback signature. */
typedef void (*uart_dfu_cb_t)(const struct uart_dfu_evt *const evt);

/** UART DFU session names. */
enum uart_dfu_sess {
	UART_DFU_SESS_IDLE,
	UART_DFU_SESS_SRV,
	UART_DFU_SESS_CLI,
	UART_DFU_SESS_SW,
	UART_DFU_SESS_COUNT
};

/** @} */

#endif  /* UART_DFU_TYPES_H__ */
