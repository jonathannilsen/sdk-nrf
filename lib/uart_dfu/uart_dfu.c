#include <zephyr.h>
#include <sys/printk.h>
#include <sys/time_units.h>
#include <sys/atomic.h>
#include <logging/log.h>
#include <string.h>
#include <drivers/uart.h>
#include <uart_dfu.h>
#include <uart_dfu_types.h>
#include <cobs.h>


/*
	TODO:
		- Add TX timeout (use driver timeout, can probably be a large one).
		- Add RX timeout (avoid driver timeout, use other timer)
		- Make flow control work with PC
		- Nice logging
		- Implement missing stuff
		- Document?
*/


/*****************************************************************************
 * Macros
 *****************************************************************************/

#define UART_DFU_MAX_FRAGMENT_SIZE

#define OFFSET_MAX_ALLOWED				INT32_MAX
#define OPCODE_NONE						-1

#define RX_WAIT_SIZE 					1
#define RX_NORMAL_SIZE					(COBS_ENCODED_SIZE(sizeof(struct uart_dfu_message)))
#define RX_WRITE_SIZE(seg_size)			(COBS_ENCODED_SIZE(sizeof(struct uart_dfu_header) + (seg_size)))
#define RX_TIMEOUT 						SYS_FOREVER_MS

#define RX_FRAME_SIZE(current_size)		(current_size + 1)

#define RX_BUF_SIZE						(COBS_MAX_BYTES + 1)
#define CLIENT_TX_BUF_SIZE				(COBS_MAX_BYTES)
#define SERVER_TX_BUF_SIZE				(COBS_ENCODED_SIZE(sizeof(struct uart_dfu_message)))

#define FLAG_CLIENT			0
#define FLAG_SERVER			1

/* FIXME: Measure correct stack size. */
#define UART_DFU_STACK_SIZE 			512


/*****************************************************************************
 * Structure definitions 
 *****************************************************************************/

enum server_rx_state {
	SERVER_RX_STATE_STOPPED = 0,
	SERVER_RX_STATE_COMMAND,
	SERVER_RX_STATE_WRITE_SEQUENCE
};

enum client_tx_state {
	CLIENT_TX_STATE_STOPPED = 0,
	CLIENT_TX_STATE_COMMAND,
	CLIENT_TX_STATE_WRITE_SEQUENCE
	// UART_DFU_CLIENT_TX_STATE_BREAK	
};

enum module_rx_state {
	RX_STATE_STOPPED = 0,
	RX_STATE_BUSY
};

enum module_tx_state {
	TX_STATE_STOPPED = 0,
	TX_STATE_BUSY
};

struct module_protocol_state {
	atomic_t rx_state;
	atomic_t tx_state;	
};

struct rx_buf_info {
	struct k_work work;
	const u8_t * buf;
	size_t offset;
	size_t len;
};

struct tx_buf_info {
	struct k_work work;
	const u8_t * buf;
	size_t len;
};

struct uart_dfu {
	const char * const 					uart_label;
	struct device * 					uart_dev;

	struct module_protocol_state 		protocol_state;
	
	u8_t 								rx_data[RX_BUF_SIZE];
	u8_t 								rx_data_decoded[COBS_MAX_DATA_BYTES];
	struct cobs_decoder 				rx_data_decoder;
	
	atomic_t 							tx_data_flags;
	struct rx_buf_info 					rx_buf_info;
	struct k_work						rx_avail_work;
	struct tx_buf_info					tx_buf_info;
	struct k_work 						tx_avail_work;
	struct k_work_q						workqueue;
	struct z_thread_stack_element * 	stack_area;

	struct uart_dfu_client *	 		client;
	struct uart_dfu_client_callbacks 	client_callbacks;
	void * 								client_context;

	struct uart_dfu_server * 			server;
	struct uart_dfu_server_callbacks	server_callbacks;
	void * 								server_context;
};


/*****************************************************************************
 * Static variables 
 *****************************************************************************/

/* FIXME: reorganize so that this is not necessary. */
static int uart_dfu_rx_start(struct uart_dfu * device);

/* NOTE: could be done in a more flexible way. */
#define __UART_DFU_INSTANCE_ENABLED(prop)	\
	DT_HAS_CHOSEN(prop)

#define UART_DFU_INSTANCE_ENABLED(idx)	\
	__UART_DFU_INSTANCE_ENABLED(uart_dfu_uart_inst_##idx)

#define __UART_DFU_INSTANCE_SUPPORTED(prop)	\
	DT_NODE_HAS_COMPAT_STATUS(DT_CHOSEN(prop), nordic_nrf_uarte, okay)

#define UART_DFU_INSTANCE_SUPPORTED(idx)	\
	__UART_DFU_INSTANCE_SUPPORTED(uart_dfu_uart_inst_##idx)

#define UART_DFU_INSTANCE_DEF(idx)															\
	static const char state_##idx##_label[] = DT_LABEL(DT_CHOSEN(uart_dfu_uart_inst_##idx));\
	K_THREAD_STACK_DEFINE(state_##idx##_stack_area, UART_DFU_STACK_SIZE);					\
	static struct uart_dfu state_##idx = {													\
		.uart_label = state_##idx##_label,													\
		.stack_area = state_##idx##_stack_area												\
	}

#if UART_DFU_INSTANCE_ENABLED(0)
#if UART_DFU_INSTANCE_SUPPORTED(0)
UART_DFU_INSTANCE_DEF(0);
#else
#error "UART DFU UART instance 0 is not enabled or is not supported."
#endif
#endif

#if UART_DFU_INSTANCE_ENABLED(1)
#if UART_DFU_INSTANCE_SUPPORTED(1)
UART_DFU_INSTANCE_DEF(1);
#else
#error "UART DFU UART instance 0 is not enabled or is not supported."
#endif
#endif

static struct uart_dfu * 			devices[] = {
#if UART_DFU_INSTANCE_ENABLED(0)
	&state_0,
#else
	NULL,
#endif
#if UART_DFU_INSTANCE_ENABLED(1)
	&state_1,
#else
	NULL
#endif
};

#define MAX_DEVICE_COUNT 			(sizeof(devices) / sizeof(struct uart_dfu *))

LOG_MODULE_REGISTER(uart_dfu, CONFIG_UART_DFU_LIBRARY_LOG_LEVEL);


/*****************************************************************************
 * Static functions 
 *****************************************************************************/

/*
 * Utility functions 
 *****************************************************************************/

static struct uart_dfu * dfu_device_get(size_t inst_idx)
{
	if (inst_idx > MAX_DEVICE_COUNT)
	{
		return NULL;
	}

	return devices[inst_idx];
}

static size_t segment_size_next_calculate(size_t current_idx, size_t total_size)
{
	size_t remaining_size = total_size - current_idx;
	return MIN(remaining_size, COBS_MAX_DATA_BYTES - sizeof(struct uart_dfu_header));
}

static inline void client_fragment_clear(struct uart_dfu_client * client)
{
	if (client != NULL)
	{
		client->fragment_buf = NULL;
		client->fragment_idx = 0;
		client->fragment_total_size = 0;
	}
}

static int pdu_encode(u8_t * buf,
					  size_t * encoded_size,
					  struct uart_dfu_header * header,
					  union uart_dfu_args * args,
					  size_t arg_size)
{
	int err;
	COBS_ENCODER_DECLARE(encoder, buf);

	/* Encode message header. */
	err = cobs_encode(&encoder, (u8_t *) header, sizeof(struct uart_dfu_header));
	if (err != 0)
	{
		return err;
	}

	/* Encode message arguments. */
	if (arg_size > 0)
	{
		err = cobs_encode(&encoder, (u8_t *) args, arg_size);
		if (err != 0)
		{
			return err;
		}	
	}

	err = cobs_encode_finish(&encoder, encoded_size);
	return err;	
}


/*
 * General TX helper functions 
 *****************************************************************************/

static int uart_dfu_send_ready(struct uart_dfu * device, atomic_t * send_flag)
{
	int err;
	u8_t * tx_buffer;
	size_t tx_size;
	int tx_timeout;

	if (!atomic_cas(&device->protocol_state.tx_state,
				    (atomic_t) TX_STATE_STOPPED,
				    (atomic_t) TX_STATE_BUSY))
	{
		/* TX already in progress. */
		return 0;
	}

	/* Choose buffer to transmit from.
	   The server has priority since it can not starve the client. */
	if (atomic_test_and_clear_bit(&device->tx_data_flags, FLAG_SERVER) &&
		device->server != NULL)
	{
		/* Server is ready to send. */
		tx_buffer = device->server->tx_data;
		tx_size = device->server->tx_size;
		if (send_flag != NULL)
		{
			*send_flag = FLAG_SERVER;
		}
	}
	else if (atomic_test_and_clear_bit(&device->tx_data_flags, FLAG_CLIENT) &&
			 device->client != NULL)
	{
		/* Client is ready to send. */
		tx_buffer = device->client->tx_data;
		tx_size = device->client->tx_size;
		if (send_flag != NULL)
		{
			*send_flag = FLAG_CLIENT;
		}
	}
	else
	{
		atomic_set(&device->protocol_state.tx_state, (atomic_t) TX_STATE_STOPPED);
		return 0;
	}

	/* TODO: use tx timeout. */	
	tx_timeout = SYS_FOREVER_MS;
	LOG_DBG("%s enabling TX (size=%u, timeout=%d)", device->uart_label, tx_size, tx_timeout);
	err = uart_tx(device->uart_dev, tx_buffer, tx_size, tx_timeout);
	if (err != 0)
	{
		atomic_set(&device->protocol_state.tx_state, (atomic_t) TX_STATE_STOPPED);
		LOG_ERR("%s error enabling TX: %d", device->uart_label, err);
		return -ECANCELED;
	}
	else
	{
		return (int) tx_size;
	}
}


/*
 * Client TX helper functions 
 *****************************************************************************/

static int client_send(struct uart_dfu * device,
					   struct uart_dfu_header * header,
					   union uart_dfu_args * args,
					   size_t arg_size)
{
	int err;

	if (device->client == NULL ||
		atomic_get(&device->client->state.tx_state) == (atomic_t) CLIENT_TX_STATE_STOPPED)
	{
		return -EINVAL;
	}

	err = pdu_encode(device->client->tx_data,
					 &device->client->tx_size,
					 header,
					 args,
					 arg_size);
	if (err != 0)
	{
		return -EINVAL;
	}
	atomic_set_bit(&device->tx_data_flags, FLAG_CLIENT);
	
	err = uart_dfu_send_ready(device, NULL);
	return err;
}

static void client_send_cancel(struct uart_dfu * device)
{
	atomic_clear_bit(&device->tx_data_flags, FLAG_CLIENT);
	device->client->tx_size = 0;
}

static int client_writec_send(struct uart_dfu * device, size_t * next_segment_size)
{
	size_t segment_size;
	struct uart_dfu_header header;
	const u8_t * segment_data;
	
	/* Constructing the message requires some special logic since this is the only
	   variable length message type. */
	memset(&header, 0, sizeof(struct uart_dfu_header));
	header.opcode = UART_DFU_OPCODE_WRITEC;
	segment_size = segment_size_next_calculate(device->client->fragment_idx,
											   device->client->fragment_total_size);
	*next_segment_size = segment_size;
	segment_data =  &device->client->fragment_buf[device->client->fragment_idx];

	return client_send(device, &header, (union uart_dfu_args *) segment_data, segment_size);
}

static int client_write_sequence_cont(struct uart_dfu * device)
{
	int err;

	/* TODO: error codes. */

	/* XXX: Should we store the sent segment size instead? */
	if (device->client->fragment_idx < device->client->fragment_total_size)
	{
		/* Continue transmitting the fragment */
		size_t segment_size;

		err = client_writec_send(device, &segment_size);
		if (err < 0)
		{
			client_fragment_clear(device->client);	
			atomic_set(&device->client->state.tx_state, (atomic_t) CLIENT_TX_STATE_STOPPED);
			return -ECANCELED;
		}
		else if (device->client->fragment_idx + segment_size >= device->client->fragment_total_size)
		{
			/* Final segment - prepare to receive reply. */
			if (!atomic_cas(&device->client->state.rx_opcode, OPCODE_NONE, UART_DFU_OPCODE_WRITEC))
			{
				client_fragment_clear(device->client);
				return -EBUSY;
			}
		}
	}
	else
	{
		/* Fragment transmission done */
		client_fragment_clear(device->client);	
		atomic_set(&device->client->state.tx_state, (atomic_t) CLIENT_TX_STATE_STOPPED);
	}

	return 0;
}


/*
 * Server TX helper functions 
 *****************************************************************************/

static int server_send(struct uart_dfu * device,
					   struct uart_dfu_header * header,
					   union uart_dfu_args * args,
					   size_t arg_size)
{
	int err;

	if (device->server == NULL)
	{
		return -EINVAL;
	}

	if (!atomic_cas(&device->server->state.tx_opcode,
					OPCODE_NONE,
					(atomic_t) header->opcode))
	{
		return -ECANCELED;
	}

	err = pdu_encode(device->server->tx_data,
						  &device->server->tx_size,
						  header,
						  args,
						  arg_size);
	if (err != 0)
	{
		(void) atomic_set(&device->server->state.tx_opcode, OPCODE_NONE);
		return err;
	}	
	atomic_set_bit(&device->tx_data_flags, FLAG_SERVER);
	
	err = uart_dfu_send_ready(device, NULL);
	return err;
}

static int client_rx_handle(struct uart_dfu * device, struct uart_dfu_message * message, size_t len)
{
	if (device->client == NULL)
	{
		return 0;
	}

	if (atomic_set(&device->client->state.rx_opcode, OPCODE_NONE) == (atomic_t) message->header.opcode)
	{
		switch (message->header.opcode)
		{
			case UART_DFU_OPCODE_INIT:
			case UART_DFU_OPCODE_DONE:
			{
				device->client_callbacks.status_callback(message->args.status.data.status,
														 device->client_context);
				break;
			}
			
			case UART_DFU_OPCODE_WRITEC:
			{
				device->client_callbacks.status_callback(message->args.status.data.status,
														 device->client_context);
				break;
			}
			case UART_DFU_OPCODE_WRITEH:
			{
				if (message->args.status.data.status == 0)
				{
					/* Success; start write sequence. */
					if (atomic_cas(&device->client->state.tx_state,
								   (atomic_t) CLIENT_TX_STATE_STOPPED,
								   (atomic_t) CLIENT_TX_STATE_WRITE_SEQUENCE))
					{
						/* XXX: handle error here? */
						return client_write_sequence_cont(device);	
					}
					else
					{
						return -ECANCELED;
					}
				}
				else
				{
					/* Failure; report error. */
					device->client_callbacks.status_callback(message->args.status.data.status,
															 device->client_context);
				}
				break;
			}	
			case UART_DFU_OPCODE_OFFSET:
			{
				if (message->args.status.data.status < 0)
				{
					device->client_callbacks.status_callback(message->args.status.data.status,
															 device->client_context);
				}
				else
				{
					device->client_callbacks.offset_callback(message->args.status.data.offset,
															 device->client_context);
				}
				break;
			}
			default:
			{
				/* Unknown opcode (this should not happen since the opcode is checked earlier). */
				return -ENOTSUP;
			}
		}
	}
	else
	{
		/* Unexpected reply. */
		return -ENOTSUP;
	}
	
	return 0;
}

static int client_tx_handle(struct uart_dfu * device)
{
	int err;
	size_t tx_size;

	/* FIXME: Get TX size from event? */
	tx_size = device->client->tx_size;
	device->client->tx_size = 0;

	if (atomic_get(&device->client->state.tx_state) == (atomic_t) CLIENT_TX_STATE_WRITE_SEQUENCE)	
	{
		device->client->fragment_idx += tx_size - COBS_ENCODED_SIZE(sizeof(struct uart_dfu_header));
		err = client_write_sequence_cont(device);
		return err;
	}
	else
	{
		atomic_set(&device->client->state.tx_state, (atomic_t) CLIENT_TX_STATE_STOPPED);	
	}

	return 0;
}

static int server_writeh_rx_handle(struct uart_dfu_server * server,
								   struct uart_dfu_message * message)
{
	u32_t fragment_total_size = message->args.writeh.fragment_total_size;
	
	if (fragment_total_size <= server->fragment_max_size)
	{
		server->fragment_idx = 0;
		server->fragment_total_size = fragment_total_size;
		return 0;
	}
	else
	{
		return -EINVAL;
	}
}

static int server_writec_rx_handle(struct uart_dfu_server * server,
								   struct uart_dfu_message * message,
								   size_t arg_size)
{
	if (server->fragment_idx + arg_size <= server->fragment_total_size)
	{
		memcpy(&server->fragment_buf[server->fragment_idx],
			   message->args.writec.data,
			   arg_size);
		server->fragment_idx += arg_size;
		if (server->fragment_idx >= server->fragment_total_size)
		{
			/* Fragment reassembly complete. */
			return 0;
		}
		else
		{
			/* Fragment not yet complete. */
			return -EMSGSIZE;
		}
	}
	else
	{
		/* Fragment size too large; reset state and return error. */
		server->fragment_idx = 0;
		server->fragment_total_size = 0;
		return -ENOMEM;
	}
}

static inline int server_rx_handle(struct uart_dfu * device,
								   struct uart_dfu_message * message,
								   size_t arg_size)
{
	int err;
	struct uart_dfu_message reply;	

	if (device->server == NULL)
	{
		return 0;
	}

	memset(&reply.header, 0, sizeof(struct uart_dfu_header));
	memset(&reply.args.status, 0, sizeof(struct uart_dfu_status_args));

	switch (message->header.opcode)
	{
		case UART_DFU_OPCODE_INIT:
		{
			reply.args.status.data.status =
				device->server_callbacks.init_callback(message->args.init.file_size,
														device->server_context);
			break;
		}
		case UART_DFU_OPCODE_WRITEH:
		{
			/* TODO: consistent error codes. */
			err = server_writeh_rx_handle(device->server, message);
			if (err == 0)
			{
				(void) atomic_set(&device->server->state.rx_state,
						          (atomic_t) SERVER_RX_STATE_WRITE_SEQUENCE);
			}	
			reply.args.status.data.status = err;
			break;
		}
		case UART_DFU_OPCODE_WRITEC:
		{
			err = server_writec_rx_handle(device->server, message, arg_size);
			if (err == 0)
			{
				/* Fragment complete - call server callback. */
				(void) atomic_set(&device->server->state.rx_state,
								  (atomic_t) SERVER_RX_STATE_COMMAND);
				
				reply.args.status.data.status =
					device->server_callbacks.write_callback(device->server->fragment_buf,
													 		 device->server->fragment_total_size,
															 device->server_context);
			}
			else if (err == -EMSGSIZE)
			{
				/* Error code signifies that the sequence is not yet done. */
				return 0;	
			}
			else
			{
				/* Error; reset RX state. */ 
				(void) atomic_set(&device->server->state.rx_state,
								  (atomic_val_t) SERVER_RX_STATE_COMMAND);
				return -ENOTSUP;
			}
			break;
		}
		case UART_DFU_OPCODE_OFFSET:
		{
			size_t offset;
			int status;
			
			status = device->server_callbacks.offset_callback(&offset, device->server_context);

			if (status == 0)
			{
				if (offset <= OFFSET_MAX_ALLOWED)
				{
					reply.args.status.data.offset = offset;
				}
				else
				{
					/* Unsupported offset value. */
					return -EINVAL;
				}
			}
			else
			{
				reply.args.status.data.status = status;
			}
			break;
		}
		case UART_DFU_OPCODE_DONE:
		{
			reply.args.status.data.status =
				device->server_callbacks.done_callback((bool) message->args.done.success,
													   device->server_context);
			break;
		}
		default:
		{
			return -EINVAL;
		}
	}

	reply.header.opcode = message->header.opcode;
	reply.header.status = 1;

	/* Send reply. */
	err = server_send(device, &reply.header, &reply.args, sizeof(struct uart_dfu_status_args));
	if (err <= 0)
	{
		LOG_ERR("%s server_send: error %d", device->uart_label, err);
		return -ECANCELED;
	}

	return err;
}

static int server_tx_handle(struct uart_dfu * device)
{
	(void) atomic_set(&device->server->state.tx_opcode, OPCODE_NONE);
	return 0;
}

static int uart_dfu_rx_parse(struct uart_dfu * device, u8_t * buf, size_t len)
{
	int err;
	struct uart_dfu_message * message;
	size_t arg_size;

	if (len < sizeof(struct uart_dfu_header))
	{
		return -EINVAL;
	}
	
	message = (struct uart_dfu_message *) buf;
	arg_size = len - sizeof(struct uart_dfu_header);

	/* Validate message length. */
	switch (message->header.opcode)
	{
		case UART_DFU_OPCODE_INIT:
		case UART_DFU_OPCODE_WRITEH:
		case UART_DFU_OPCODE_OFFSET:
		case UART_DFU_OPCODE_DONE:
		{
			if (arg_size != UART_DFU_ARGUMENT_SIZE)
			{
				/* All these PDUs have the same fixed length. */
				return -EINVAL;
			}
			break;
		}
		case UART_DFU_OPCODE_WRITEC:
		{
			if (arg_size == 0)
			{
				/* Empty segments are disallowed. */
				return -EINVAL;
			}
			break;
		}
		default:
		{
			/* Unknown opcode. */
			return -EINVAL;
		}
	}

	/* FIXME: error codes below should be valid. */

	if (message->header.status)
	{
		err = client_rx_handle(device, message, arg_size);
		if (err < 0 && device->client_callbacks.error_callback != NULL)
		{
			device->client_callbacks.error_callback(err, device->client_context);
		}
	}
	else
	{
		err = server_rx_handle(device, message, arg_size);
		if (err < 0 && device->server_callbacks.error_callback != NULL)
		{
			device->server_callbacks.error_callback(err, device->server_context);
		}
	}

	return err;
}

static void server_rx_params_get(struct uart_dfu_server * server, u32_t * rx_size, u32_t * rx_timeout)
{
	enum server_rx_state rx_state;

	if (server == NULL)
	{
		*rx_size = 0;
		*rx_timeout = SYS_FOREVER_MS;
		return;
	}

	rx_state = (enum server_rx_state) atomic_get(&server->state.rx_state);

	switch (rx_state)
	{
		case SERVER_RX_STATE_COMMAND:
		{
			*rx_size = RX_NORMAL_SIZE;
			break;
		}
		case SERVER_RX_STATE_WRITE_SEQUENCE:
		{
			*rx_size = RX_WRITE_SIZE(segment_size_next_calculate(server->fragment_idx,
																 server->fragment_total_size));
			break;
		}
		default:
		{
			*rx_size = 0;
			break;
		}
	}

	*rx_timeout = RX_TIMEOUT;
}

static void client_rx_params_get(struct uart_dfu_client * client, u32_t * rx_size, u32_t * rx_timeout)
{
	if (client == NULL)
	{
		*rx_size = 0;
		*rx_timeout = SYS_FOREVER_MS;
		return;
	}

	if (atomic_get(&client->state.rx_opcode) != OPCODE_NONE)
	{
		*rx_size = RX_NORMAL_SIZE;
	}
	else
	{
		*rx_size = 0;
	}

	*rx_timeout = RX_TIMEOUT;
}

static int uart_dfu_rx_next(struct uart_dfu * device, atomic_t * recv_flag)
{
	u32_t server_rx_size;
	u32_t server_rx_timeout;
	u32_t client_rx_size;
	u32_t client_rx_timeout;
	u32_t rx_size;
	u32_t rx_timeout;

	server_rx_params_get(device->server, &server_rx_size, &server_rx_timeout);
	client_rx_params_get(device->client, &client_rx_size, &client_rx_timeout);

	if (server_rx_size > 0 && recv_flag != NULL)
	{
		atomic_set_bit(recv_flag, FLAG_SERVER);
	}
	if (client_rx_size > 0 && recv_flag != NULL)
	{
		atomic_set_bit(recv_flag, FLAG_CLIENT);
	}

	rx_size = MAX(client_rx_size, server_rx_size);
	rx_timeout = MAX(client_rx_timeout, server_rx_timeout);
	
	if (rx_size == 0)
	{
		return 0;
	}

	if (cobs_decode_in_frame(&device->rx_data_decoder))
	{
		if (cobs_decode_current_size(&device->rx_data_decoder) < rx_size)
		{
			rx_size -= cobs_decode_current_size(&device->rx_data_decoder);
		}
		else
		{
			rx_size = RX_WAIT_SIZE;
		}
	}

	if (atomic_cas(&device->protocol_state.rx_state,
				   (atomic_val_t) RX_STATE_STOPPED,
				   (atomic_val_t) RX_STATE_BUSY))
	{
		LOG_DBG("%s: enabling RX (size=%u, timeout=%d)", device->uart_label, rx_size, rx_timeout);
		return uart_rx_enable(device->uart_dev, device->rx_data, rx_size, rx_timeout);
	}
	else
	{
		return -EBUSY;
	}
}

static int uart_dfu_rx_start(struct uart_dfu * device)
{
	if (atomic_get(&device->protocol_state.rx_state) == (atomic_t) RX_STATE_STOPPED)
	{
		return uart_dfu_rx_next(device, NULL);
	}
	else
	{
		return 0;
	}
}

static int client_transaction_reserve(struct uart_dfu_client * client, u8_t opcode)
{
	/* Reserve TX. */
	if (!atomic_cas(&client->state.tx_state,
					(atomic_t) CLIENT_TX_STATE_STOPPED,
					(atomic_t) CLIENT_TX_STATE_COMMAND))
	{
		// FIXME
		printk("client tx busy.\n");
		return -EBUSY;
	}

	/* Reserve RX. */
	if (!atomic_cas(&client->state.rx_opcode,
				    OPCODE_NONE,
					(atomic_t) opcode))
	{
		atomic_set(&client->state.tx_state, (atomic_t) CLIENT_TX_STATE_STOPPED);
		// FIXME
		printk("client rx busy.\n");
		return -EBUSY;
	}

	return 0;
}

static void client_transaction_cancel(struct uart_dfu_client * client)
{
	/* Reset TX/RX state. */
	atomic_set(&client->state.rx_opcode, OPCODE_NONE);
	atomic_set(&client->state.tx_state, (atomic_t) CLIENT_TX_STATE_STOPPED);
}


/*
 * UART event handlers 
 *****************************************************************************/

static void tx_buf_process(struct k_work * work)
{
	int err;
	struct tx_buf_info * info;
	struct uart_dfu * device;

	info = CONTAINER_OF(work, struct tx_buf_info, work);
	device = CONTAINER_OF(info, struct uart_dfu, tx_buf_info);
	
	if (device->client != NULL && info->buf == device->client->tx_data)
	{
		err = client_tx_handle(device);
		if (err != 0)
		{
			LOG_ERR("%s: error handling client TX: %d", device->uart_label, err);
			if (device->client_callbacks.error_callback != NULL)
			{
				device->client_callbacks.error_callback(err, device->client_context);
			}
		}
		
	}
	else if (device->server != NULL && info->buf == device->server->tx_data)
	{
		err = server_tx_handle(device);
		if (err != 0)
		{
			LOG_ERR("%s: error handling server TX: %d", device->uart_label, err);
			if (device->server_callbacks.error_callback != NULL)
			{
				device->server_callbacks.error_callback(err, device->server_context);
			}
		}
	}
	else
	{
		/* Client/server was removed in the middle of TX. */
		return;
	}
}

static void tx_avail_process(struct k_work * work)
{
	int err;
	struct uart_dfu * device;
	atomic_t send_flag;

	device = CONTAINER_OF(work, struct uart_dfu, tx_avail_work);
	
	/* XXX: Is there a problem in updating this state here? */
	atomic_set(&device->protocol_state.tx_state, (atomic_t) TX_STATE_STOPPED);
	
	err = uart_dfu_send_ready(device, &send_flag);
	if (err < 0)
	{
		LOG_ERR("%s: error %d while trying to send data.", device->uart_label, err);
		if (atomic_test_bit(&send_flag, FLAG_CLIENT) &&
			device->client_callbacks.error_callback != NULL)
		{
			device->client_callbacks.error_callback(err, device->client_context);
		}
		else if (atomic_test_bit(&send_flag, FLAG_SERVER) &&
				 device->server_callbacks.error_callback != NULL)
		{
			device->server_callbacks.error_callback(err, device->server_context);
		}
	}

	if (device->client != NULL && atomic_get(&device->client->state.rx_opcode) != OPCODE_NONE)
	{
		/* XXX: return error? */
		(void) uart_dfu_rx_start(device);
	}
}

static void uart_tx_done_handle(struct uart_dfu * device, struct uart_event_tx * evt)
{
	LOG_DBG("%s: TX (len=%u)", device->uart_label, evt->len);

	device->tx_buf_info.buf = evt->buf;
	device->tx_buf_info.len = evt->len;

	k_work_submit_to_queue(&device->workqueue, &device->tx_buf_info.work);
	k_work_submit_to_queue(&device->workqueue, &device->tx_avail_work);
}

static void uart_tx_aborted_handle(struct uart_dfu * device, struct uart_event_tx * evt)
{
	atomic_set(&device->protocol_state.tx_state, (atomic_t) TX_STATE_STOPPED);
	k_work_submit_to_queue(&device->workqueue, &device->tx_avail_work);
	
	/* TODO: Possibly start processing work. Do once TX timeout is added. */
}

static void rx_buf_process(struct k_work * work)
{
	int err;
	struct rx_buf_info * info;
	struct uart_dfu * device;
	size_t offset;
	size_t bound;
	size_t decoded_size;

	info = CONTAINER_OF(work, struct rx_buf_info, work);
	device = CONTAINER_OF(info, struct uart_dfu, rx_buf_info);
	
	offset = info->offset;
	bound = offset + info->len;

	LOG_DBG("%s: RX process (offset=%u, len=%u)", device->uart_label, info->offset, info->len);

	while (offset < bound)
	{
		err = cobs_decode(&device->rx_data_decoder,
						  &decoded_size,
						  &info->buf[info->offset],
						  &offset,
						  bound - offset);
		if (err == -EMSGSIZE)
		{
			LOG_DBG("%s: message not yet complete.", device->uart_label);
		}
		if (err == 0)
		{
			err = uart_dfu_rx_parse(device, device->rx_data_decoded, decoded_size);
			if (err >= 0)
			{
				LOG_DBG("%s: successfully parsed PDU.", device->uart_label);
			}
			else
			{
				LOG_HEXDUMP_ERR(&info->buf[info->offset], info->len, "Encoded data (full):");
				LOG_HEXDUMP_ERR(device->rx_data_decoded, decoded_size, "Decoded data:");
			}
		}
		else	
		{
			LOG_ERR("%s: error %d while decoding data.", device->uart_label, err);
			if (device->server != NULL && device->server_callbacks.error_callback != NULL)
			{
				device->server_callbacks.error_callback(-ENOTSUP, device->server_context);
			}
			if (device->client != NULL && device->client_callbacks.error_callback != NULL)
			{
				device->client_callbacks.error_callback(-ENOTSUP, device->client_context);
			}
		}
	}

	LOG_DBG("%s: finished processing received data.", device->uart_label);
}

static void rx_avail_process(struct k_work * work)
{
	int err;
	struct uart_dfu * device;
	atomic_t recv_flags;

	device = CONTAINER_OF(work, struct uart_dfu, rx_avail_work);

	/* XXX: Is there a problem in updating this state here? */
	atomic_set(&device->protocol_state.rx_state, (atomic_t) RX_STATE_STOPPED);
	
	/* Resume reception if applicable. */
	err = uart_dfu_rx_next(device, &recv_flags);
	if (err != 0 && err != -EINVAL)
	{
		LOG_ERR("%s: error %d while starting RX.", device->uart_label, err);

		/* FIXME: what happens to the client/server RX state? */
		if (atomic_test_bit(&recv_flags, FLAG_SERVER) &&
			device->server_callbacks.error_callback != NULL)
		{
			device->server_callbacks.error_callback(-EBUSY, device->server_context);
		}
		if (atomic_test_bit(&recv_flags, FLAG_CLIENT) &&
			device->client_callbacks.error_callback != NULL)
		{
			device->client_callbacks.error_callback(-EBUSY, device->client_context);
		}	
	}
}

static void uart_rx_rdy_handle(struct uart_dfu * device, struct uart_event_rx * evt)
{
	LOG_DBG("%s: RX (offset=%u, len=%u)", device->uart_label, evt->offset, evt->len);

	device->rx_buf_info.buf = evt->buf;
	device->rx_buf_info.offset = evt->offset;
	device->rx_buf_info.len = evt->len;

	k_work_submit_to_queue(&device->workqueue, &device->rx_buf_info.work);
}

static void uart_rx_buf_request_handle(struct uart_dfu * device)
{
	/* Double buffering is not used for now, so no implementation is needed. */
}

static void uart_rx_buf_released_handle(struct uart_dfu * device, struct uart_event_rx_buf * evt)
{
	/* Double buffering is not used for now, so no implementation is needed. */
}

static void uart_rx_disabled_handle(struct uart_dfu * device)
{
	k_work_submit_to_queue(&device->workqueue, &device->rx_avail_work);
}

static void uart_rx_stopped_handle(struct uart_dfu * device, struct uart_event_rx_stop * evt)
{
	/* TODO */
}

static void uart_async_cb(struct uart_event * evt, void * user_data)
{
	struct uart_dfu * device;

	device = (struct uart_dfu *) user_data;

	switch (evt->type)
	{
		case UART_TX_DONE:
		{
			LOG_DBG("%s: UART_TX_DONE", device->uart_label);
			uart_tx_done_handle(device, &evt->data.tx);
			break;
		}
		case UART_TX_ABORTED:
		{
			LOG_DBG("%s: UART_TX_ABORTED", device->uart_label);
			uart_tx_aborted_handle(device, &evt->data.tx);
			break;
		}
		case UART_RX_RDY:
		{
			LOG_DBG("%s: UART_RX_RDY", device->uart_label);
			uart_rx_rdy_handle(device, &evt->data.rx);
			break;
		}
		case UART_RX_BUF_REQUEST:
		{
			LOG_DBG("%s: UART_RX_BUF_REQUEST", device->uart_label);
			uart_rx_buf_request_handle(device);
			break;
		}
		case UART_RX_BUF_RELEASED:
		{
			LOG_DBG("%s: UART_RX_BUF_RELEASED", device->uart_label);
			uart_rx_buf_released_handle(device, &evt->data.rx_buf);
			break;
		}
		case UART_RX_DISABLED:
		{
			LOG_DBG("%s: UART_RX_DISABLED", device->uart_label);
			uart_rx_disabled_handle(device);
			break;
		}
		case UART_RX_STOPPED:
		{
			LOG_DBG("%s: UART_RX_STOPPED", device->uart_label);
			uart_rx_stopped_handle(device, &evt->data.rx_stop);
			break;
		}
		default:
		{
			break;
		}
	}
}


/*****************************************************************************
 * API functions 
 *****************************************************************************/

int uart_dfu_init(size_t inst_idx)
{
	int err;
	struct uart_dfu * device;

	device = dfu_device_get(inst_idx);
	if (device == NULL)
	{
		return -EINVAL;
	}

	printk("Initializing UART DFU on %s.\n", device->uart_label);

	device->uart_dev = device_get_binding(device->uart_label);
	if (device->uart_dev == NULL)
	{
		return -EACCES;
	}

	err = uart_callback_set(device->uart_dev, uart_async_cb, device);
	if (err != 0)
	{
		device->uart_dev = NULL;
		return -EIO;
	}

	k_work_q_start(&device->workqueue,
				   device->stack_area,
				   UART_DFU_STACK_SIZE,
				   CONFIG_UART_DFU_LIBRARY_THREAD_PRIO);

	memset(&device->protocol_state, 0, sizeof(struct module_protocol_state));
	(void) cobs_decoder_init(&device->rx_data_decoder, device->rx_data_decoded);	
	device->tx_data_flags = ATOMIC_INIT(0);
	k_work_init(&device->rx_buf_info.work, rx_buf_process);
	k_work_init(&device->rx_avail_work, rx_avail_process);
	k_work_init(&device->tx_buf_info.work, tx_buf_process);
	k_work_init(&device->tx_avail_work, tx_avail_process);
	device->client = NULL;
	memset(&device->client_callbacks, 0, sizeof(struct uart_dfu_client_callbacks));
	device->client_context = NULL;
	device->server = NULL;
	memset(&device->server_callbacks, 0, sizeof(struct uart_dfu_server_callbacks));
	device->server_context = NULL;

	printk("Initialized UART DFU on %s.\n", device->uart_label);

	return 0;
}

static int uart_dfu_sys_init(struct device * dev)
{
	ARG_UNUSED(dev);

	int err;

	for (size_t inst_idx = 0; inst_idx < MAX_DEVICE_COUNT; inst_idx++)
	{
		if (devices[inst_idx] != NULL)
		{
			err = uart_dfu_init(inst_idx);
			if (err != 0)
			{
				return err;
			}
		}
	}

	return 0;
}

int uart_dfu_uninit(size_t inst_idx)
{
#if 0
	int err_code = 0;

	/* TODO: Have another look at this function. */

	if (protocol_state.rx_state != UART_DFU_RX_STATE_STOPPED ||
		protocol_state.tx_state != UART_DFU_TX_STATE_STOPPED ||
		protocol_state.tx_pending)
	{
		return -EBUSY;
	}

	if (uart_dev != NULL)
	{
		err_code = uart_callback_set(uart_dev, NULL, NULL);
	}
	if (err_code == 0)
	{
		uart_dev = NULL;
		memset(&protocol_state, 0, sizeof(struct uart_dfu_state));
		(void) cobs_decode_reset(&rx_data_decoder);
		client = NULL;
		server = NULL;
	}

	return err_code;
#endif
	return 0;
}

int uart_dfu_client_init(struct uart_dfu_client * dfu_client)
{
	if (dfu_client == NULL)
	{
		return -EINVAL; 
	}

	dfu_client->fragment_buf = NULL;
	dfu_client->fragment_idx = 0;
	dfu_client->fragment_total_size = 0;
	dfu_client->tx_size = 0;
	atomic_set(&dfu_client->state.tx_state, (atomic_val_t) CLIENT_TX_STATE_STOPPED);
	atomic_set(&dfu_client->state.rx_opcode, OPCODE_NONE);

	return 0;
}

int uart_dfu_client_set(size_t inst_idx,
						struct uart_dfu_client * dfu_client,
						struct uart_dfu_client_callbacks * callbacks,
						void * context)
{
	struct uart_dfu * device;
	
	if (dfu_client == NULL					||
		callbacks == NULL 					||
		callbacks->status_callback == NULL 	||
		callbacks->offset_callback == NULL)
	{
		return -EINVAL; 
	}

	device = dfu_device_get(inst_idx);
	if (device == NULL)
	{
		return -EINVAL;
	}

	/* TODO: Check for ongoing client ops. */

	device->client = dfu_client;
	device->client_callbacks = *callbacks;
	device->client_context = context;
	
	return 0;
}

int uart_dfu_client_clear(size_t inst_idx)
{
	return -ENOTSUP;
}

int uart_dfu_client_init_send(size_t inst_idx, size_t file_size)
{
	int err;
	struct uart_dfu * device;
	struct uart_dfu_message message;

	device = dfu_device_get(inst_idx);
	if (device == NULL)
	{
		return -EINVAL;
	}

	if (device->uart_dev == NULL || device->client == NULL)
	{
		return -EACCES;
	}

	if (file_size > INT32_MAX)
	{
		return -ENOMEM;
	}

	err = client_transaction_reserve(device->client, UART_DFU_OPCODE_INIT);
	if (err != 0)
	{
		return err;
	}

	memset(&message, 0, sizeof(struct uart_dfu_message));
	message.header.opcode = UART_DFU_OPCODE_INIT;
	message.args.init.file_size = (u32_t) file_size;

	err = client_send(device,
					  &message.header,
					  &message.args,
					  sizeof(struct uart_dfu_init_args));

	if (err >= 0)
	{
		/* Start RX if it is not already running. */
		err = uart_dfu_rx_start(device);
	}
	if (err != 0)	
	{
		client_send_cancel(device);
		client_transaction_cancel(device->client);
	}
	
	return err;
}

int uart_dfu_client_write_send(size_t inst_idx,
							   const u8_t * const fragment_buf,
							   size_t fragment_size)
{
	int err;
	struct uart_dfu * device;
	struct uart_dfu_message message;

	device = dfu_device_get(inst_idx);
	if (device == NULL)
	{
		return -EINVAL;
	}
	
	if (device->uart_dev == NULL || device->client == NULL)
	{
		return -EACCES;
	}
	
	if (fragment_buf == NULL || fragment_size == 0)
	{
		return -EINVAL;
	}

	if (fragment_size > INT32_MAX)
	{
		return -ENOMEM;
	}

	err = client_transaction_reserve(device->client, UART_DFU_OPCODE_WRITEH);
	if (err != 0)
	{
		return err;
	}

	device->client->fragment_buf = fragment_buf;
	device->client->fragment_idx = 0;
	device->client->fragment_total_size = fragment_size;
	
	memset(&message, 0, sizeof(struct uart_dfu_message));
	message.header.opcode = UART_DFU_OPCODE_WRITEH;
	message.args.writeh.fragment_total_size = fragment_size;

	err = client_send(device,
					  &message.header,
					  &message.args,
					  sizeof(struct uart_dfu_writeh_args));

	if (err >= 0)
	{
		/* Start RX if it is not already running. */
		err = uart_dfu_rx_start(device);
		if (err != 0)
		{
			printk("uart_dfu_rx_start %d\n", err);
		}
	}
	if (err != 0)	
	{
		client_send_cancel(device);
		client_transaction_cancel(device->client);
	}
	
	return err;
}

int uart_dfu_client_offset_send(size_t inst_idx)
{
	int err;
	struct uart_dfu * device;
	struct uart_dfu_message message;

	device = dfu_device_get(inst_idx);
	if (device == NULL)
	{
		return -EINVAL;
	}

	if (device->uart_dev == NULL || device->client == NULL)
	{
		return -EACCES;
	}

	err = client_transaction_reserve(device->client, UART_DFU_OPCODE_OFFSET);
	if (err != 0)
	{
		return err;
	}

	memset(&message, 0, sizeof(struct uart_dfu_message));
	message.header.opcode = UART_DFU_OPCODE_OFFSET;

	err = client_send(device,
					  &message.header,
					  &message.args,
					  sizeof(struct uart_dfu_offset_args));

	if (err >= 0)
	{
		/* Start RX if it is not already running. */
		err = uart_dfu_rx_start(device);
	}
	if (err != 0)	
	{
		client_send_cancel(device);
		client_transaction_cancel(device->client);
	}
	
	return err;
}

int uart_dfu_client_done_send(size_t inst_idx, bool successful)
{
	int err;
	struct uart_dfu * device;
	struct uart_dfu_message message;

	device = dfu_device_get(inst_idx);
	if (device == NULL)
	{
		return -EINVAL;
	}

	if (device->uart_dev == NULL || device->client == NULL)
	{
		return -EACCES;
	}

	err = client_transaction_reserve(device->client, UART_DFU_OPCODE_DONE);
	if (err != 0)
	{
		return err;
	}

	memset(&message, 0, sizeof(struct uart_dfu_message));
	message.header.opcode = UART_DFU_OPCODE_DONE;
	message.args.done.success = successful;

	err = client_send(device,
					  &message.header,
					  &message.args,
					  sizeof(struct uart_dfu_done_args));

	if (err >= 0)
	{
		/* Start RX if it is not already running. */
		err = uart_dfu_rx_start(device);
	}
	if (err != 0)	
	{
		client_send_cancel(device);
		client_transaction_cancel(device->client);
	}
	
	return err;
}

int uart_dfu_client_stop(size_t inst_idx)
{
	return -ENOTSUP;
}

int uart_dfu_server_init(struct uart_dfu_server * dfu_server,
						u8_t * fragment_buf,
						size_t fragment_max_size)
{
	if (dfu_server == NULL 		||
		fragment_buf == NULL 	||
		fragment_max_size == 0)
	{
		return -EINVAL;
	}
	
	memset(dfu_server, 0, sizeof(struct uart_dfu_server));
	dfu_server->fragment_buf = fragment_buf;
	dfu_server->fragment_idx = 0;
	dfu_server->fragment_max_size = fragment_max_size;
	dfu_server->tx_size = 0;
	atomic_set(&dfu_server->state.rx_state, (atomic_t) SERVER_RX_STATE_STOPPED);
	atomic_set(&dfu_server->state.tx_opcode, OPCODE_NONE);
	atomic_set(&dfu_server->state.tx_opcode, OPCODE_NONE);

	return 0;
}

int uart_dfu_server_set(size_t inst_idx,
						struct uart_dfu_server * dfu_server,
						struct uart_dfu_server_callbacks * callbacks,
						void * context)
{
	struct uart_dfu * device;

	if (dfu_server == NULL 					||
		callbacks == NULL 					||
		callbacks->init_callback == NULL 	||
		callbacks->write_callback == NULL	||
		callbacks->offset_callback == NULL 	||
		callbacks->done_callback == NULL)
	{
		return -EINVAL;
	}
	
	device = dfu_device_get(inst_idx);
	if (device == NULL)
	{
		return -EINVAL;
	}

	/* TODO: Check for ongoing server activity. */
	device->server = dfu_server;
	device->server_callbacks = *callbacks;
	device->server_context = context;

	return 0;
}

int uart_dfu_server_clear(size_t inst_idx)
{
	return -ENOTSUP;
}

int uart_dfu_server_enable(size_t inst_idx)
{
	int err;
	struct uart_dfu * device;

	device = dfu_device_get(inst_idx);
	if (device == NULL)
	{
		return -EINVAL;
	}

	if (device->uart_dev == NULL || device->server == NULL)
	{
		return -EACCES;
	}

	if (atomic_cas(&device->server->state.rx_state,
				   (atomic_t) SERVER_RX_STATE_STOPPED,
				   (atomic_t) SERVER_RX_STATE_COMMAND))
	{
		err = uart_dfu_rx_start(device);
		if (err != 0)
		{
			atomic_set(&device->server->state.rx_state,
					   (atomic_t) SERVER_RX_STATE_STOPPED);
		}
		return err;
	}
	else
	{
		/* Already started. */
		return 0;
	}
}

int uart_dfu_server_disable(size_t inst_idx)
{
	struct uart_dfu * device;
	
	device = dfu_device_get(inst_idx);
	if (device == NULL)
	{
		return -EINVAL;
	}

	if (device->uart_dev == NULL || device->server == NULL)
	{
		return -EACCES;
	}
	/* TODO */

	return -ENOTSUP;
}


/*****************************************************************************
 * System initialization hooks 
 *****************************************************************************/

#ifdef CONFIG_UART_DFU_SYS_INIT
SYS_INIT(uart_dfu_sys_init, APPLICATION, CONFIG_UART_DFU_INIT_PRIORITY);
#endif