/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_DFU_TYPES_H__
#define UART_DFU_TYPES_H__

#include <zephyr/types.h>


/*****************************************************************************
 * Macros
 *****************************************************************************/

#define UART_DFU_OPCODE_INIT			0x00
#define UART_DFU_OPCODE_WRITEH			0x01
#define UART_DFU_OPCODE_WRITEC			0x02
#define UART_DFU_OPCODE_OFFSET			0x03
#define UART_DFU_OPCODE_DONE			0x04

#define UART_DFU_ARGUMENT_SIZE  		4

/* TODO: Make status into a field ?
    00: No status
    01: API status
    10: Protocol status
    11: Offset
*/


/*****************************************************************************
 * Structure definitions 
 *****************************************************************************/

struct uart_dfu_header {
	u8_t opcode : 3;
	u8_t status : 1;
	u8_t rfu 	: 4;
} __packed;

struct uart_dfu_init_args {
	u32_t file_size;
} __packed;

struct uart_dfu_writeh_args {
	u32_t fragment_total_size;
} __packed;

struct uart_dfu_writec_args {
    u8_t data[0];  /* Variable length data. */
} __packed;

struct uart_dfu_offset_args {
	u8_t padding[4];
} __packed;

struct uart_dfu_done_args {
	u8_t padding1[3];
	u8_t padding2	: 7;
	u8_t success 	: 1;
} __packed;

struct uart_dfu_status_args {
	union {
		u32_t offset;
		s32_t status;
	} data;
} __packed;

union uart_dfu_args {
    struct uart_dfu_status_args status;
    struct uart_dfu_init_args init;
    struct uart_dfu_writeh_args writeh;
    struct uart_dfu_writec_args writec;
    struct uart_dfu_done_args done;
};

struct uart_dfu_message {
	struct uart_dfu_header header;
    union uart_dfu_args args;
} __packed;

#endif  /* UART_DFU_TYPES_H__ */
