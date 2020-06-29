#include <zephyr.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <string.h>
#include <lte_uart_dfu.h> 
#include <dfu/dfu_target.h>
#include "dfu_target_mcuboot.h"
#include <drivers/flash.h>

//TODO: Many of the stuctures/variables could be declared in lte_uart_dfu.h

// TODO: Figure out what the optimal stack size is
#define UART_HANDLER_STACK_SIZE 500

K_THREAD_STACK_DEFINE(uart_handler_stack, UART_HANDLER_STACK_SIZE);
static struct k_thread uart_handler_thread;

#define UART_INPUT_BUF_SIZE 8192
#define UART_OUTPUT_BUF_SIZE 40

static struct input_buffer_type{
    u8_t buffer[UART_INPUT_BUF_SIZE];
    int offs; 
};

static struct input_buffer_type input_buffer;

static struct output_buffer_type{
    u8_t buffer[UART_OUTPUT_BUF_SIZE];
    int offs;
};


static struct output_buffer_type output_buffer;

static void append_to_buffer(void* append_data, int len){
    memcpy(output_buffer.buffer + output_buffer.offs, append_data, len);
    output_buffer.offs+=len;
}

static const char magic_start[] = "xogq";
static const char magic_stop[] =  "foqs";
static const int mcuboot_magic = 0x96f3b83d;

static struct device *uart_dev;

K_SEM_DEFINE(lte_uart_sem, 0, 1);

enum dfu_targ_func{
	INIT,
	WRITE,
	OFFSET,
	DONE
}; 

static int send_data()
{
	if (output_buffer.offs == 0) {
		return 0;
	}
	for(int i = 0; i < output_buffer.offs; i++){
		uart_poll_out(uart_dev, output_buffer.buffer[i]);
	}

	return 0;
}

static void send_error(enum dfu_targ_func dfu_func, int error){
    
    append_to_buffer((void*)magic_start, sizeof(magic_start) - 1);
    append_to_buffer(&dfu_func, sizeof(dfu_func));
    append_to_buffer(&error, sizeof(error));
    append_to_buffer((void*)magic_stop, sizeof(magic_stop)-1);
    
    send_data();
    output_buffer.offs = 0;
}

static struct write_func{
	char buf[6000]; //TODO: Implement a pointer here to save memory
	int len;
	int ret_val;
};

static struct done_func{
	bool successful;
	int ret_val;
};

static struct init_func{
	size_t file_size;
	int ret_val;
};

static struct offset_func{
	size_t offset;
	int ret_val;
};

static struct lte_uart_type{
	struct write_func write;
	struct done_func done;
	struct  offset_func offset;
	struct init_func init;
};

static struct lte_uart_type lte_uart;

static void unused_func(enum dfu_target_evt_id evt)
{
	ARG_UNUSED(evt);
}

static void uart_cb(struct device *x)
{
    static int bytes_recv = 0;
	u8_t byte;
	int rx;

	uart_irq_update(x);

	if (!uart_irq_rx_ready(x)) {
		return;
	}
	while (true) {
		rx = uart_fifo_read(x, &byte, 1);

		if (bytes_recv < sizeof(input_buffer.buffer)) {
			input_buffer.buffer[bytes_recv] = byte;
		}
		if (strncmp(&input_buffer.buffer[bytes_recv-3], magic_stop, 4) == 0) {
            input_buffer.offs = bytes_recv;
            bytes_recv = 0;
			printk("Magic stop received,  current transmission is complete\n");
			k_sem_give(&lte_uart_sem);
			break;
		}
		if (rx != 1) {
			break;
		}
		
		bytes_recv++;
	}
}

static void parse_init(void){
	lte_uart.init.file_size = (size_t)input_buffer.buffer[5];
}

static void parse_write(void){
	static bool download_started = false;
	
	int lngth;
	memcpy(&lngth,&input_buffer.buffer[5],sizeof(lngth)); //TODO: Do memcpy rigth into lte_uart.write.len
	
	lte_uart.write.len = lngth;
	printk("Received fragment of size %d\n", lte_uart.write.len);
	
	memcpy(lte_uart.write.buf, &input_buffer.buffer[9], lngth);
	
	if(!download_started){
		// First fragment received, change magic header value from uart_magic to mcuboot_magic, such that it is compatible with mcuboot
		download_started = true;
		memcpy(lte_uart.write.buf,&mcuboot_magic, 4);
	}
	
}

static void parse_done(void){
	lte_uart.done.successful = (int)input_buffer.buffer[5];
}

static void parse_input_data(void){
	int err;
	int dfu_func = (int)input_buffer.buffer[4];
    size_t offs_get;

	switch (dfu_func){
    case INIT:
		parse_init();
		
		err = dfu_target_mcuboot_init(lte_uart.init.file_size, unused_func);

        send_error(INIT, err);
		
		break;

    case WRITE:
		parse_write();
		
		err = dfu_target_mcuboot_write(lte_uart.write.buf, lte_uart.write.len);

        send_error(WRITE, err);
		break;

	case OFFSET:		
		dfu_target_mcuboot_offset_get(&offs_get);

        /*
            TODO: Should maybe create an own function send_offset(), but this should work
            fine as well
        */
        send_error(OFFSET, offs_get);

        break;

	case DONE:
		parse_done();
		
		err = dfu_target_mcuboot_done(lte_uart.done.successful);

        send_error(DONE, err);

      	break;	
    default:
		break;
	}	
	
}


static void uart_input_handler(void *dummy1, void *dummy2, void *dummy3)
{
    while (1) {

		k_sem_take(&lte_uart_sem, K_FOREVER);
        parse_input_data();
	}
}

int lte_uart_dfu_start(void){
    /*
        TODO: Look into if the thread (uart_input_handler()) should be 
		created in the init function instead and suspended right away
	    and the resumed in this function? 
		
		Look at nrf\subsys\net\lib\download_client\src\download_client.c
        how it's done

    */
   
    /*
        TODO: Figure out what the optimal priority to set here
    */
    k_thread_create(&uart_handler_thread, uart_handler_stack,
        K_THREAD_STACK_SIZEOF(uart_handler_stack),
        uart_input_handler, NULL, NULL, NULL,
        K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
        k_thread_name_set(&uart_handler_thread, "uart handler");
    return 0;
}

int lte_uart_dfu_init(void)
{
    uart_dev = device_get_binding(CONFIG_LTE_UART_DFU_UART_LABEL);
	uart_irq_callback_set(uart_dev, uart_cb);
	uart_irq_rx_enable(uart_dev); // Should this be called in lte_uart_dfu_start()?
	printk("LTE UART DFU library initialized\n"); 

    output_buffer.offs = 0; 

    /* 
        TODO: Maybe implement some error checking on the stuff above and return
        an error if anything fails
    */
    return 0; 

}
