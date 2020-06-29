/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_DFU_UTIL_H__
#define UART_DFU_UTIL_H__

#include <sys/atomic.h>
#include <uart_dfu.h>
#include <uart_dfu_types.h>
#include <cobs.h>


/*****************************************************************************
* Macros
*****************************************************************************/

#define OFFSET_MAX_ALLOWED	INT32_MAX
#define OPCODE_NONE		-1
#define OPCODE_ANY		-2

#define PDU_SIZE(arg_len)	(sizeof(struct uart_dfu_hdr) + (arg_len))
#define ARG_SIZE(pdu_len)	((pdu_len) - sizeof(struct uart_dfu_hdr))


/*****************************************************************************
* Static functions
*****************************************************************************/

static inline size_t seg_size_next(size_t idx, size_t len)
{
	size_t rem_size = len - idx;
	return MIN(rem_size, COBS_MAX_DATA_BYTES - sizeof(struct uart_dfu_hdr));
}

#endif // UART_DFU_UTIL_H__
