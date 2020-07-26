/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
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
#define COBS_ENCODED_SIZE(_sz)  ((_sz) + COBS_OVERHEAD_BYTES)


enum cobs_decoder_state {
    COBS_DECODER_STATE_WAIT,
    COBS_DECODER_STATE_DECODE,
    COBS_DECODER_STATE_INVALID
};

struct cobs_decoder {
    u8_t * buf_out;
    size_t out_idx;
    size_t next_delimiter;
    enum cobs_decoder_state state;
};

struct cobs_encoder {
    u8_t * buf_out;
    size_t out_idx;
    size_t last_delimiter;
};


#define COBS_DECODER_DECLARE(name, buf)     \
    struct cobs_decoder name = {            \
        .buf_out = buf,                     \
        .out_idx = 0,                       \
        .next_delimiter = 0,                \
        .state = COBS_DECODER_STATE_WAIT    \
    }

#define COBS_ENCODER_DECLARE(name, buf) \
    struct cobs_encoder name = {        \
        .buf_out = buf,                 \
        .out_idx = 1,                   \
        .last_delimiter = 0             \
    }


static inline int cobs_encoder_reset(struct cobs_encoder * encoder)
{
    if (encoder == NULL)
    {
        return -EINVAL;
    }

    /* Reset encoder state. */
    encoder->out_idx = 1;
    encoder->last_delimiter = 0; 

    return 0;
}

static inline int cobs_encoder_init(struct cobs_encoder * encoder, u8_t * buf_out)
{
    if (encoder == NULL || buf_out == NULL)
    {
        return -EINVAL;
    }

    encoder->buf_out = buf_out;

    return cobs_encoder_reset(encoder);
}

static inline int cobs_encode(struct cobs_encoder * encoder,
                              const u8_t * const buf_in,
                              size_t length_in)
{
    if (encoder == NULL             ||
        encoder->buf_out == NULL    ||
        buf_in == NULL)
    {
        return -EINVAL;
    }
    
    if (encoder->out_idx + length_in + 1 > COBS_MAX_BYTES)
    {
        /* Total length of encoded frame is greater than maximum allowed size. */
        return -ENOMEM;
    }
    
    for (size_t in_idx = 0; in_idx < length_in; in_idx++, encoder->out_idx++)
    {
        if (buf_in[in_idx] == COBS_DELIMITER)
        {
            /* Escape values that are equal to the delimiter. */
            encoder->buf_out[encoder->last_delimiter] = encoder->out_idx;
            encoder->last_delimiter = encoder->out_idx;
        }
        else
        {
            /* Copy other values as is. */
            encoder->buf_out[encoder->out_idx] = buf_in[in_idx];
        }
    }
    
    return 0;
}

static inline int cobs_encode_finish(struct cobs_encoder * encoder,
                                     size_t * length_out)
{

    if (encoder == NULL             ||
        encoder->buf_out == NULL    ||
        length_out == NULL)
    {
        return -EINVAL;
    }

    /* Insert delimiter and update the delimiter pointer. */
    encoder->buf_out[encoder->out_idx] = COBS_DELIMITER;
    encoder->buf_out[encoder->last_delimiter] = encoder->out_idx;
    *length_out = encoder->out_idx + 1;   

    return cobs_encoder_reset(encoder);
}

static inline int cobs_decoder_reset(struct cobs_decoder * decoder)
{
    if (decoder == NULL)
    {
        return -EINVAL;
    }

    decoder->out_idx = 0;
    decoder->next_delimiter = 0;
    decoder->state = COBS_DECODER_STATE_WAIT;

    return 0;
}

static inline int cobs_decoder_init(struct cobs_decoder * decoder, u8_t * buf_out)
{
    if (decoder == NULL || buf_out == NULL)
    {
        return -EINVAL;
    }

    decoder->buf_out = buf_out;
    return cobs_decoder_reset(decoder);
}

static inline int cobs_decode(struct cobs_decoder * decoder,
                              size_t * length_out,
                              const u8_t * const buf_in,
                              size_t * offset_in,
                              size_t length_in)
{
    if (decoder == NULL                 ||
        decoder->buf_out == NULL        ||
        length_out == NULL              ||
        buf_in == NULL                  || 
        offset_in == NULL               ||
        *offset_in >= length_in)
    {
        return -EINVAL;
    }

    bool complete = false;
    size_t in_idx;
    
    /* Decode loop. */
    for (in_idx = *offset_in; in_idx < length_in && !complete; in_idx++)
    {
        switch (decoder->state)
        {
            case COBS_DECODER_STATE_WAIT:
            {
                if (buf_in[in_idx] != COBS_DELIMITER)
                {
                    decoder->next_delimiter = (size_t) buf_in[in_idx];
                    decoder->state = COBS_DECODER_STATE_DECODE;
                }
                else
                {
                    /* First byte was a delimiter, ignore */
                }
                break;
            }
            case COBS_DECODER_STATE_DECODE:
            {
                if (buf_in[in_idx] != COBS_DELIMITER)
                {
                    if (decoder->out_idx == decoder->next_delimiter - 1)
                    {
                        /* Encoded delimiter reached. */
                        decoder->buf_out[decoder->out_idx] = COBS_DELIMITER;
                        if (buf_in[in_idx] > decoder->out_idx + 1)
                        {
                            decoder->next_delimiter = buf_in[in_idx];
                        }
                        else
                        {
                            /* Invalid frame encoding. */
                            (void) cobs_decoder_reset(decoder);
                            decoder->state = COBS_DECODER_STATE_INVALID;
                        }
                    }
                    else
                    {
                        decoder->buf_out[decoder->out_idx] = buf_in[in_idx];
                    }
                    
                    decoder->out_idx++;
                }
                else
                {
                    /* Delimiter detected before buffer end. */
                    decoder->state = COBS_DECODER_STATE_WAIT;
                    complete = true; 
                    break;
                }

                if (decoder->out_idx > COBS_MAX_DATA_BYTES)
                {
                    (void) cobs_decoder_reset(decoder);
                    decoder->state = COBS_DECODER_STATE_INVALID;
                }
                break;
            }
            case COBS_DECODER_STATE_INVALID:
            {
                /* Clearing invalid frame - look for delimiter to resume decoding. */
                if (buf_in[in_idx] == COBS_DELIMITER)
                {
                    decoder->state = COBS_DECODER_STATE_WAIT;
                }
                break;
            }
        }
    }

    *offset_in = in_idx;

    if (complete)
    {
        /* Complete frame decoded. */
        *length_out = decoder->out_idx;
        (void) cobs_decoder_reset(decoder);

        return 0;
    }
    else
    {
        /* Incomplete frame. */
        *length_out = 0;
        if (decoder->state != COBS_DECODER_STATE_INVALID)
        {
            return -EMSGSIZE;
        }
        else
        {
            return -ENOTSUP;
        }
    }
}

static inline bool cobs_decode_in_frame(struct cobs_decoder * decoder)
{
    return decoder->state == COBS_DECODER_STATE_DECODE;
}

static inline size_t cobs_decode_current_size(struct cobs_decoder * decoder)
{
    return decoder->out_idx;
}

#endif  /* COBS_H__ */