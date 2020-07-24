#ifndef UART_DFU_H__
#define UART_DFU_H__

#include <stddef.h>
#include <zephyr/types.h>


/*****************************************************************************
 * Structure definitions 
 *****************************************************************************/

/* TODO: Formalization of error codes sent to error_callback. */

struct uart_dfu_client_callbacks {
	void 	(*status_callback)(int status);
	void 	(*offset_callback)(size_t offset);
	void 	(*available_callback)(void);
	void 	(*error_callback)(int error);
};

struct uart_dfu_client {
	const u8_t * fragment_buf;
	size_t 	fragment_idx;
	size_t  fragment_total_size;
	struct uart_dfu_client_callbacks callbacks;
};

struct uart_dfu_server_callbacks {
	int 	(*init_callback)(size_t file_size);
	int 	(*write_callback)(const u8_t * const fragment_buf,
						   	  size_t fragment_size);
	int 	(*offset_callback)(size_t * offset);
	int 	(*done_callback)(bool successful);
	void 	(*error_callback)(int error);
};

struct uart_dfu_server {
	u8_t * 	fragment_buf;
	size_t	fragment_max_size; 
	size_t 	fragment_idx;
	size_t  fragment_total_size;
	struct uart_dfu_server_callbacks callbacks;
};


/*****************************************************************************
 * API functions 
 *****************************************************************************/

int uart_dfu_init(void);

int uart_dfu_uninit(void);

int uart_dfu_client_set(struct uart_dfu_client * dfu_client,
						struct uart_dfu_client_callbacks * callbacks);

int uart_dfu_client_clear(void);

int uart_dfu_client_init_send(size_t file_size);

int uart_dfu_client_write_send(const u8_t * const fragment_buf,
							   size_t fragment_size);

int uart_dfu_client_offset_send(void);

int uart_dfu_client_done_send(bool successful);

int uart_dfu_client_stop(void);

int uart_dfu_server_set(struct uart_dfu_server * dfu_server,
						u8_t * fragment_buf,
						size_t fragment_max_size,
						struct uart_dfu_server_callbacks * callbacks);

int uart_dfu_server_clear(void);

int uart_dfu_server_enable(void);

int uart_dfu_server_disable(void);

#endif  /* UART_DFU_H__ */