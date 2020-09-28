/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef UART_BLOB_UTIL_H__
#define UART_BLOB_UTIL_H__

#include <sys/atomic.h>
#include <uart_cobs.h>
#include "uart_blob_types.h"

#define OFFSET_MAX_ALLOWED	INT32_MAX
#define OPCODE_NONE		-1
#define OPCODE_ANY		-2

#define PDU_SIZE(arg_len)	(sizeof(struct uart_blob_hdr) + (arg_len))
#define ARG_SIZE(pdu_len)	((pdu_len) - sizeof(struct uart_blob_hdr))

static inline size_t seg_size_next(size_t idx, size_t len)
{
	size_t rem_size = len - idx;
	return MIN(rem_size, UART_COBS_MAX_PAYLOAD_LEN -
			     sizeof(struct uart_blob_hdr));
}

#endif // UART_BLOB_UTIL_H__
