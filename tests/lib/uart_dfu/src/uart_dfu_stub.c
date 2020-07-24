/* needed here so the static device_get_binding does not get renamed */
#include <device.h>
#include <drivers/uart.h>

/* Forward declare functions to be replaced. */
struct device *device_get_binding_stub(const char *name);
int uart_callback_set_stub(struct device *dev,
				    	   uart_callback_t callback,
				    	   void *user_data);
int uart_tx_stub(struct device *dev,
                 const u8_t *buf,
                 size_t len,
                 s32_t timeout);
int uart_rx_enable_stub(struct device *dev,
                        u8_t *buf,
                        size_t len,
                        s32_t timeout);
int uart_rx_disable_stub(struct device *dev);
int uart_rx_buf_rsp_stub(struct device *dev,
                         u8_t *buf,
                         size_t len);


/* Redefine the functions for this source file. */
#define device_get_binding device_get_binding_stub
#define uart_callback_set uart_callback_set_stub
#define uart_tx uart_tx_stub
#define uart_rx_enable uart_rx_enable_stub
#define uart_rx_disable uart_rx_disable_stub
#define uart_rx_buf_rsp uart_rx_buf_rsp_stub


/* file itself */
#include "uart_dfu.c"
