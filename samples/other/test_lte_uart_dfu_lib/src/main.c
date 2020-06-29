#include <zephyr.h>
#include <sys/printk.h>
#include <string.h>
#include <lte_uart_dfu.h>


void main(void)
{
	printk("test_lte_uart_dfu_lib sample started\n");
	lte_uart_dfu_init();
	lte_uart_dfu_start();
}
