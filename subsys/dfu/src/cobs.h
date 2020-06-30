/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */

#ifndef COBS_H__
#define COBS_H__

#include <stddef.h>
#include <errno.h>

#include <zephyr/types.h>


#define COBS_DELIMITER          0x00
#define COBS_OVERHEAD_BYTES     2
#define COBS_MAX_BYTES          255
#define COBS_MAX_DATA_BYTES     (COBS_MAX_BYTES - COBS_OVERHEAD_BYTES)


static inline int cobs_encode(u8_t * buf_out,
                              size_t * length_out,
                              const u8_t * const buf_in,
                              size_t length_in)
{
    if (buf_out == NULL     ||
        length_out == NULL  ||
        buf_in == NULL)
    {
        return -EINVAL;
    }

    size_t last_delimiter = 0;
    size_t out_idx;

    for (out_idx = 1;
         out_idx < length_in && out_idx < COBS_MAX_DATA_BYTES - 1;
         out_idx++)
    {
        if (buf_in[out_idx - 1] == COBS_DELIMITER)
        {
            buf_out[last_delimiter] = out_idx;
            last_delimiter = out_idx;
        }
        else
        {
            buf_out[out_idx] = buf_in[out_idx - 1];
        }
    }
    buf_out[out_idx] = COBS_DELIMITER;

    if (last_delimiter == 0)
    {
        buf_out[0] = out_idx;
    }

    *length_out = out_idx + 1;
    
    return 0;
}


static inline int cobs_decode(u8_t * buf_out,
                              u8_t * length_out,
                              const u8_t * const buf_in,
                              size_t length_in)
{
    if (buf_out == NULL                     ||
        length_out == NULL                  ||
        buf_in == NULL                      ||
        length_in < COBS_OVERHEAD_BYTES)
    {
        return -EINVAL;
    }

    size_t next_delimiter = (size_t) buf_in[0];
    size_t in_idx;

    for (in_idx = 1;
         in_idx < length_in && in_idx < COBS_MAX_BYTES;
         in_idx++)
    {
        if (buf_in[in_idx] != COBS_DELIMITER)
        {
            if (in_idx == next_delimiter)
            {
                buf_out[in_idx - 1] = COBS_DELIMITER;
                next_delimiter = in_idx;
            }
            else
            {
                buf_out[in_idx - 1] = buf_in[in_idx];
            }
        }
        else
        {
            break;
        }
    }

    if (buf_in[in_idx] != COBS_DELIMITER ||
        next_delimiter != in_idx - 1)
    {
        return -ENOTSUP;
    }

    *length_out = (in_idx + 1) - COBS_OVERHEAD_BYTES;
    return 0;
}


#endif  /* COBS_H__ */