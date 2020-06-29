#ifndef DFU_TARGET_UART_H__
#define DFU_TARGET_UART_H__

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
    TODO: Add explanatory text for the functions below
*/

bool dfu_target_uart_identify(const void *const buf);

int dfu_target_uart_init(size_t file_size, dfu_target_callback_t cb);

int dfu_target_uart_offset_get(size_t *offset);

int dfu_target_uart_write(const void *const buf, size_t len);

int dfu_target_uart_done(bool successful);

#endif /* DFU_TARGET_UART_H__ */