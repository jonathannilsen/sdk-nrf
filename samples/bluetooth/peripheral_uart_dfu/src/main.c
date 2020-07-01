#include <zephyr.h>
#include <logging/log.h>
#include <string.h>
#include <uart_dfu.h>
#include <uart_dfu_target_server.h>

LOG_MODULE_REGISTER(peripheral_uart_dfu, LOG_LEVEL_DBG);


void main(void)
{
	LOG_INF("peripheral_uart_dfu starting.");
#if 0
	int err_code;

	LOG_INF("Initializing peripheral UART.");

	err_code = uart_dfu_init();
	if (err_code != 0)
	{
		LOG_ERR("Failed to initialize UART DFU.");
		return;
	}

	err_code = uart_dfu_target_server_init();
	if (err_code != 0)
	{
		LOG_ERR("Failed to initialize UART DFU server.");
		return;
	}

	err_code = uart_dfu_target_server_enable();
	if (err_code != 0)
	{
		LOG_ERR("Failed to enable UART DFU server.");
		return;
	}

	LOG_INF("Sample started.\n");

	while (1)	
	{
	}
#endif

	LOG_INF("Sample started.");
}
