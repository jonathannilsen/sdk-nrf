#include <zephyr.h>
#include <dfu/dfu_target.h>
#include <pm_config.h>
#include <logging/log.h>
#include <sys/printk.h>
#include <drivers/uart.h>
#include <string.h>

#define UART_HEADER_MAGIC  0x85f3d83a

#define UART_INPUT_BUF_SIZE 100
#define UART_OUTPUT_BUF_SIZE 8192

K_SEM_DEFINE(dfu_targ_uart_sem, 0, 1);

//TODO: Many of the stuctures/variables could be declared in dfu_target_uart.h

static struct output_buffer_type{
    u8_t buffer[UART_OUTPUT_BUF_SIZE];
    int offs;
};

static struct input_buffer_type{
	u8_t buffer[UART_INPUT_BUF_SIZE];
	int offs;
};

static struct output_buffer_type output_buffer;
static struct input_buffer_type input_buffer;

static struct device *uart_dev;

static struct dfu_target_uart_input{
	int active_func;
	int error;
	int offset;
};

static struct dfu_target_uart_input input_fields;

static const char magic_start[] = "xogq";
static const char magic_stop[] =  "foqs";

enum dfu_targ_func{
	INIT,
	WRITE,
	OFFSET,
	DONE
}; 

typedef void (*dfu_target_callback_t)(enum dfu_target_evt_id evt_id);

static void append_to_out_buffer(void* append_data, int len){
	//TODO: Implement a method of checking if the buffer overflows
    memcpy(output_buffer.buffer + output_buffer.offs, append_data, len);
    output_buffer.offs+=len;
}

//TODO: Make this function return an error on failure
static int send_data(void)
{
	if (output_buffer.offs == 0) {
		return 0;
	}
	for(int i = 0; i < output_buffer.offs; i++){
		uart_poll_out(uart_dev, output_buffer.buffer[i]);
	}
	output_buffer.offs = 0;
	return 0;
}

/*
	TODO: Resolve the compiler warning "note: expected 'void *' but argument is of type 'const void * const'"
*/
static void send_fragment(void *buf, size_t size){

    append_to_out_buffer((void*)magic_start, sizeof(magic_start) - 1);
	enum dfu_targ_func write = WRITE;
    append_to_out_buffer(&write, sizeof(write));
	append_to_out_buffer(&size, sizeof(size));
    append_to_out_buffer(buf, size);
    append_to_out_buffer((void*)magic_stop, sizeof(magic_stop) - 1);

    send_data();
}

static void send_init(size_t file_size){

	append_to_out_buffer((void*)magic_start, sizeof(magic_start) - 1);
	enum dfu_targ_func init = INIT;
    append_to_out_buffer(&init, sizeof(init));	
	append_to_out_buffer(&file_size, sizeof(file_size));
	append_to_out_buffer((void*)magic_stop, sizeof(magic_stop) - 1);
	
	send_data();
}

static void send_offset(void){

	append_to_out_buffer((void*)magic_start, sizeof(magic_start) - 1);
	enum dfu_targ_func offset = OFFSET;
    append_to_out_buffer(&offset, sizeof(offset));
	append_to_out_buffer((void*)magic_stop, sizeof(magic_stop) - 1);

	send_data();
}

static void send_done(bool success){

	append_to_out_buffer((void*)magic_start, sizeof(magic_start) - 1);	
	enum dfu_targ_func done = DONE;
    append_to_out_buffer(&done, sizeof(done));
	append_to_out_buffer(&success, sizeof(success));	
	append_to_out_buffer((void*)magic_stop, sizeof(magic_stop) -1);

	send_data();
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
		// TODO: If buffer overflows an error should be returned
		if (bytes_recv < sizeof(input_buffer.buffer)) {
			input_buffer.buffer[bytes_recv] = byte;
		}
		if (strncmp(&input_buffer.buffer[bytes_recv-3], magic_stop, 4) == 0) {
            input_buffer.offs = bytes_recv;
            bytes_recv = 0;
			printk("Magic stop received, current transmission is completemagic stop received, current transmission is complete\n");
			k_sem_give(&
			dfu_targ_uart_sem);
			break;
		}

		if (rx != 1) {
			break;
		}	
		bytes_recv++;
	}
}

bool dfu_target_uart_identify(const void *const buf)
{
	return *((const u32_t *)buf) == UART_HEADER_MAGIC;
}

static void parse_input(void){
	input_fields.active_func = (int)input_buffer.buffer[4];
	if(input_fields.active_func != OFFSET){
		input_fields.error = (int)input_buffer.buffer[5];
	}else{
		input_fields.offset = (int)input_buffer.buffer[5];
	}
}

int dfu_target_uart_init(size_t file_size, dfu_target_callback_t cb)
{
	printk("dfu_target_uart_init() called\n");
	ARG_UNUSED(cb);

	/*
		TODO: If device_get_binding() fails, an error should be returned and the function should fail
	*/
	uart_dev = device_get_binding("UART_2");
    if (!uart_dev) {
        printk("ERROR: Could not get UART 2\n");
	}

	uart_irq_callback_set(uart_dev, uart_cb);
	uart_irq_rx_enable(uart_dev);

	send_init(file_size);

	/*
		The program will wait here until it receives a message that ends with magic_stop

		TODO: Might be smart to implement some kind of timeout that triggers after x
		seconds, and then this function returns an error
	*/

	k_sem_take(&dfu_targ_uart_sem, K_FOREVER);

	parse_input();

	//	TODO: Should I check this and possibly return an error?
	/*if(input_fields.active_func == INIT){
		printk("9160: Correct function INIT returned the error %d\n", input_fields.error);
	} else{
		printk("9160: Did not receive correct function INIT, received %d instead\n", input_fields.active_func);
	}*/

	return input_fields.error;
}

int dfu_target_uart_offset_get(size_t *out)
{
	printk("dfu_target_uart_offset_get() called\n");

	send_offset();

	/*
		The program will wait here until it receives a message that ends with magic_stop

		TODO: Maybe you should implement some kind of timeout that triggers after e.g. 5 
		seconds, and then this function returns an error
	*/
	
	k_sem_take(&dfu_targ_uart_sem, K_FOREVER);

	parse_input();
	*out = input_fields.offset;
	
	//TODO: Should I check this and possibly return an error?
	/*if(input_fields.active_func == OFFSET){
		printk("9160: Correct function (offset) returned an error\n");
	} else{
		printk("9160: Wrong function returned, not offset\n");
	}*/

	return 0;
}

/*
	TODO: Resolve the warning warning: passing argument 1 of 'send_fragment' discards 'const' qualifier from pointer target type"
*/

int dfu_target_uart_write(const void *const buf, size_t len)
{
	printk("dfu_target_uart_write() called\n");

	send_fragment(buf, len);

	/*
		The program will wait here until it receives a message that ends with magic_stop

		TODO: Maybe you should implement some kind of timeout that triggers after e.g. 5 
		seconds, and then this function returns an error
	*/
	
	k_sem_take(&dfu_targ_uart_sem, K_FOREVER);
	
	parse_input();

	//TODO: Should I check this and possibly return an error?
	/*if(input_fields.active_func == WRITE){
		printk("9160: Correct function (write) returned the error %d\n", input_fields.error);
	} else{
		printk("9160: Wrong function returned, not write\n" );
	}*/

	return input_fields.error;
}

int dfu_target_uart_done(bool successful)
{
	printk("dfu_target_uart_write() called, UART DFU transmission complete\n");
	send_done(successful);

	/*
		The program will wait here until it receives a message that ends with magic_stop

		TODO: Maybe you should implement some kind of timeout that triggers after e.g. 5 
		seconds, and then this function returns an error
	*/
	

	k_sem_take(&dfu_targ_uart_sem, K_FOREVER);

	parse_input();

	//TODO: Should this be checked and possibly return an error?
	/*if(input_fields.active_func == DONE){
		printk("9160: Correct function (done) returned the error %d\n", input_fields.error);
	} else{
		printk("9160: Wrong function returned, not done\n");
	}*/

	return input_fields.error;
}