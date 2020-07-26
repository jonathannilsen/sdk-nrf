#ifndef UART_DFU_TARGET_SERVER_H__
#define UART_DFU_TARGET_SERVER_H__

#include <zephyr.h>
#include <sys/atomic.h>
#include <uart_dfu.h>


struct uart_dfu_target_server {
    struct uart_dfu_server server;
    size_t inst_idx;
    atomic_t file_size;
    atomic_t initialized;
    u8_t fragment_buffer[CONFIG_UART_DFU_TARGET_SERVER_MAX_FRAGMENT_SIZE];
};


int uart_dfu_target_server_init(struct uart_dfu_target_server * server,
                                size_t inst_idx);
int uart_dfu_target_server_enable(struct uart_dfu_target_server * server);
int uart_dfu_target_server_disable(struct uart_dfu_target_server * server);


#endif /* UART_DFU_TARGET_SERVER_H__ */