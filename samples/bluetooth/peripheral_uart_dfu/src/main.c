#include <zephyr.h>
#include <logging/log.h>
#include <string.h>
#include <uart_dfu.h>
#include <uart_dfu_target_server.h>

static struct uart_dfu_target_server target_server;

LOG_MODULE_REGISTER(peripheral_uart_dfu, LOG_LEVEL_DBG);


void main(void)
{
	int err_code;

	LOG_INF("Initializing peripheral_uart_dfu.");

	err_code = uart_dfu_target_server_init(&target_server, 0);
	if (err_code != 0)
	{
		LOG_ERR("Failed to initialize UART DFU server.");
		return;
	}

	err_code = uart_dfu_target_server_enable(&target_server);
	if (err_code != 0)
	{
		LOG_ERR("Failed to enable UART DFU server.");
		return;
	}

	LOG_INF("Sample started.");
}
