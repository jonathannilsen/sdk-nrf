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

#define TX_FLAG_CLIENT_READY			0
#define TX_FLAG_SERVER_READY			1


/*****************************************************************************
 * Structure definitions 
 *****************************************************************************/

enum client_tx_state {
	CLIENT_TX_STATE_STOPPED = 0,
	CLIENT_TX_STATE_COMMAND,
	CLIENT_TX_STATE_WRITE_SEQUENCE
	// CLIENT_TX_STATE_BREAK	
};

struct client_protocol_state {
	atomic_t rx_opcode;
	atomic_t tx_state;
	size_t segment_size;
};

enum server_rx_state {
	SERVER_RX_STATE_STOPPED = 0,
	SERVER_RX_STATE_WAIT,
	SERVER_RX_STATE_COMMAND,
	SERVER_RX_STATE_WRITE_SEQUENCE
};

struct server_protocol_state {
	atomic_t rx_state;
	atomic_t tx_opcode;	
};

enum module_rx_state {
	RX_STATE_STOPPED = 0,
	RX_STATE_WAIT,
	RX_STATE_ACTIVE
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

/*
struct uart_dfu {
	struct device * 				uart_dev;
	struct module_protocol_state 	protocol_state;
	
	u8_t 							rx_data[RX_BUF_SIZE];
	u8_t 							rx_data_decoded[COBS_MAX_DATA_BYTES];
	struct cobs_decoder 			rx_data_decoder;
	
	atomic_t 						tx_data_flags;
	u8_t 							tx_data_client[CLIENT_TX_BUF_SIZE];
	size_t 							tx_size_client;
	u8_t 							tx_data_server[SERVER_TX_BUF_SIZE];
	size_t 							tx_size_server;
	
	struct rx_buf_info 				rx_buf_info;
	struct k_work					rx_avail_work;
	struct tx_buf_info				tx_buf_info;
	struct k_work 					tx_avail_work;

	struct uart_dfu_client *	 	client;
	struct client_protocol_state 	client_protocol_state;
	struct uart_dfu_server * 		server;
	struct server_protocol_state 	server_protocol_state;
};
*/

/*****************************************************************************
 * Static variables 
 *****************************************************************************/

/* TODO: Move to header to support multiple instances? */

static struct device * 				uart_dev;
static struct module_protocol_state protocol_state; 

static u8_t 						rx_data[RX_BUF_SIZE];
static u8_t 						rx_data_decoded[COBS_MAX_DATA_BYTES];
COBS_DECODER_DECLARE(rx_data_decoder, rx_data_decoded);

static atomic_t 					tx_data_flags = ATOMIC_INIT(0);
static u8_t							tx_data_client[CLIENT_TX_BUF_SIZE];
static size_t 						tx_size_client;
static u8_t 						tx_data_server[SERVER_TX_BUF_SIZE];
static size_t 						tx_size_server;

static struct uart_dfu_client * 	client = NULL;
static struct client_protocol_state client_protocol_state;
static struct uart_dfu_server * 	server = NULL;
static struct server_protocol_state server_protocol_state;

static struct rx_buf_info 			rx_buf_info;
static struct k_work				rx_avail_work;
static struct tx_buf_info			tx_buf_info;
static struct k_work 				tx_avail_work;

/* FIXME: configurable log level. */
LOG_MODULE_REGISTER(uart_dfu, LOG_LEVEL_ERR);


/*****************************************************************************
 * Static functions 
 *****************************************************************************/

static size_t segment_size_next_calculate(size_t current_idx, size_t total_size)
{
	size_t remaining_size = total_size - current_idx;
	return MIN(remaining_size, COBS_MAX_DATA_BYTES - sizeof(struct uart_dfu_header));
}

static inline void client_fragment_clear(void)
{
	if (client != NULL)
	{
		client->fragment_buf = NULL;
		client->fragment_idx = 0;
		client->fragment_total_size = 0;
	}
}

static int uart_dfu_encode(u8_t * buf,
						   size_t * encoded_size,
						   struct uart_dfu_header * header,
						   union uart_dfu_args * args,
						   size_t arg_size)
{
	int err_code;

	COBS_ENCODER_DECLARE(encoder, buf);

	/* Encode message header. */
	err_code = cobs_encode(&encoder, (u8_t *) header, sizeof(struct uart_dfu_header));
	if (err_code != 0)
	{
		return err_code;
	}

	/* Encode message arguments. */
	if (arg_size > 0)
	{
		err_code = cobs_encode(&encoder, (u8_t *) args, arg_size);
		if (err_code != 0)
		{
			return err_code;
		}	
	}

	err_code = cobs_encode_finish(&encoder, encoded_size);
	return err_code;	
}

static int uart_dfu_send_ready(void)
{
	int err_code;
	u8_t * tx_buffer;
	size_t tx_size;
	int tx_timeout;

	if (!atomic_cas(&protocol_state.tx_state,
				    (atomic_t) TX_STATE_STOPPED,
				    (atomic_t) TX_STATE_BUSY))
	{
		return -EEXIST;  /* XXX: correct error code? */
	}

	/* Choose buffer to transmit from.
	   The server has priority since it can not starve the client. */
	if (atomic_test_and_clear_bit(&tx_data_flags, TX_FLAG_SERVER_READY))
	{
		/* Server is ready to send. */
		tx_buffer = tx_data_server;
		tx_size = tx_size_server;
	}
	else if (atomic_test_and_clear_bit(&tx_data_flags, TX_FLAG_CLIENT_READY))
	{
		/* Client is ready to send. */
		tx_buffer = tx_data_client;
		tx_size = tx_size_client;
	}
	else
	{
		atomic_set(&protocol_state.tx_state, (atomic_t) TX_STATE_STOPPED);
		return 0;
	}
	
	tx_timeout = SYS_FOREVER_MS;
	LOG_DBG("Enabling TX (size=%u, timeout=%d)", tx_size, tx_timeout);
	err_code = uart_tx(uart_dev, tx_buffer, tx_size, tx_timeout);
	if (err_code != 0)
	{
		atomic_set(&protocol_state.tx_state, (atomic_t) TX_STATE_STOPPED);
		LOG_ERR("Error enabling TX: %d", err_code);
		return err_code;	
	}
	else
	{
		return (int) tx_size;
	}
}

static int client_send(struct uart_dfu_header * header,
					   union uart_dfu_args * args,
					   size_t arg_size)
{
	int err_code;

	if (client == NULL ||
		atomic_get(&client_protocol_state.tx_state) == (atomic_t) CLIENT_TX_STATE_STOPPED)
	{
		return -EINVAL;
	}

	err_code = uart_dfu_encode(tx_data_client,
							   &tx_size_client,
							   header,
							   args,
							   arg_size);
	if (err_code != 0)
	{
		return err_code;
	}
	atomic_set_bit(&tx_data_flags, TX_FLAG_CLIENT_READY);
	
	err_code = uart_dfu_send_ready();
	return err_code;
}

static void client_send_cancel(void)
{
	atomic_clear_bit(&tx_data_flags, TX_FLAG_CLIENT_READY);
	tx_size_client = 0;
}

static int client_writec_send(size_t * next_segment_size)
{
	size_t segment_size;
	struct uart_dfu_header header;
	
	/* Constructing the message requires some special logic since this is the only
	   variable length message type. */
	memset(&header, 0, sizeof(struct uart_dfu_header));
	header.opcode = UART_DFU_OPCODE_WRITEC;
	segment_size = segment_size_next_calculate(client->fragment_idx, client->fragment_total_size);
	*next_segment_size = segment_size;
	return client_send(&header,
					   (union uart_dfu_args *) &client->fragment_buf[client->fragment_idx],
					   segment_size);
}

static int client_write_sequence_cont(void)
{
	int err_code;

	/* TODO: error codes. */

	/* XXX: Should we store the sent segment size instead? */
	if (client->fragment_idx < client->fragment_total_size)
	{
		/* Continue transmitting the fragment */
		size_t segment_size;

		err_code = client_writec_send(&segment_size);
		if (err_code < 0)
		{
			client_fragment_clear();	
			atomic_set(&client_protocol_state.tx_state, (atomic_t) CLIENT_TX_STATE_STOPPED);
			/* TODO: call error callback. */
		}
		else if (client->fragment_idx + segment_size >= client->fragment_total_size)
		{
			/* Final segment - prepare to receive reply. */
			if (!atomic_cas(&client_protocol_state.rx_opcode, OPCODE_NONE, UART_DFU_OPCODE_WRITEC))
			{
				/* TODO: call error callback. */
				k_oops();
			}
		}
	}
	else
	{
		/* Fragment transmission done */
		client_fragment_clear();	
	}

	return 0;
}

static int server_send(struct uart_dfu_header * header,
					   union uart_dfu_args * args,
					   size_t arg_size)
{
	int err_code;

	if (server == NULL)
	{
		return -EINVAL;
	}

	if (!atomic_cas(&server_protocol_state.tx_opcode,
					OPCODE_NONE,
					(atomic_t) header->opcode))
	{
		return -EBUSY;
	}

	err_code = uart_dfu_encode(tx_data_server,
							   &tx_size_server,
							   header,
							   args,
							   arg_size);
	if (err_code != 0)
	{
		(void) atomic_set(&server_protocol_state.tx_opcode, OPCODE_NONE);
		return err_code;
	}	
	atomic_set_bit(&tx_data_flags, TX_FLAG_SERVER_READY);
	
	err_code = uart_dfu_send_ready();
	return err_code;
}

static inline int client_rx_handle(struct uart_dfu_message * message, size_t len)
{
	if (client == NULL)
	{
		return 0;
	}

	if (atomic_set(&client_protocol_state.rx_opcode, OPCODE_NONE) == (atomic_t) message->header.opcode)
	{
		switch (message->header.opcode)
		{
			case UART_DFU_OPCODE_INIT:
			case UART_DFU_OPCODE_DONE:
			{
				client->callbacks.status_callback(message->args.status.data.status);
				break;
			}
			
			case UART_DFU_OPCODE_WRITEC:
			{
				/* TODO: special handling. */

				client->callbacks.status_callback(message->args.status.data.status);
				break;
			}
			case UART_DFU_OPCODE_WRITEH:
			{
				if (message->args.status.data.status == 0)
				{
					/* Success; start write sequence. */
					if (atomic_cas(&client_protocol_state.tx_state,
								   (atomic_t) CLIENT_TX_STATE_STOPPED,
								   (atomic_t) CLIENT_TX_STATE_WRITE_SEQUENCE))
					{
						/* XXX: handle error here? */
						return client_write_sequence_cont();	
					}
					else
					{
						/* TODO: handle error. */
						k_oops();
					}
				}
				else
				{
					/* Failure; report error. */
					client->callbacks.status_callback(message->args.status.data.status);
				}
				break;
			}	
			case UART_DFU_OPCODE_OFFSET:
			{
				if (message->args.status.data.status < 0)
				{
					client->callbacks.status_callback(message->args.status.data.status);
				}
				else
				{
					client->callbacks.offset_callback(message->args.status.data.offset);
				}
				break;
			}
			default:
			{
				/* Unknown opcode (this should not happen). */
				return -EINVAL;
			}
		}
	}
	else
	{
		/* Unexpected reply. */
		return -EINVAL;
	}
	
	return 0;
}

static int client_tx_handle(void)
{
	int err_code;
	size_t tx_size;

	tx_size = tx_size_client;
	tx_size_client = 0;

	if (atomic_get(&client_protocol_state.tx_state) == (atomic_t) CLIENT_TX_STATE_WRITE_SEQUENCE)	
	{
		client->fragment_idx += tx_size - COBS_ENCODED_SIZE(sizeof(struct uart_dfu_header));
		err_code = client_write_sequence_cont();
		/* TODO: Handle error. */
		if (err_code != 0)
		{
			k_oops();
		}
	}
	else
	{
		atomic_set(&client_protocol_state.tx_state, (atomic_t) CLIENT_TX_STATE_STOPPED);	
	}

	return 0;
}

static int server_writeh_rx_handle(struct uart_dfu_message * message)
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

static int server_writec_rx_handle(struct uart_dfu_message * message, size_t arg_size)
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

static inline int server_rx_handle(struct uart_dfu_message * message, size_t arg_size)
{
	int err_code;
	struct uart_dfu_message reply;	

	if (server == NULL)
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
				server->callbacks.init_callback(message->args.init.file_size);
			break;
		}
		case UART_DFU_OPCODE_WRITEH:
		{
			/* TODO: consistent error codes. */
			err_code = server_writeh_rx_handle(message);
			if (err_code == 0)
			{
				(void) atomic_set(&server_protocol_state.rx_state,
						          (atomic_t) SERVER_RX_STATE_WRITE_SEQUENCE);
			}	
			reply.args.status.data.status = err_code;
			break;
		}
		case UART_DFU_OPCODE_WRITEC:
		{
			err_code = server_writec_rx_handle(message, arg_size);
			if (err_code == 0)
			{
				/* Fragment complete - call server callback. */
				(void) atomic_set(&server_protocol_state.rx_state,
								  (atomic_t) SERVER_RX_STATE_COMMAND);
				
				reply.args.status.data.status =
					server->callbacks.write_callback(server->fragment_buf,
													 server->fragment_total_size);
			}
			else if (err_code == -EMSGSIZE)
			{
				/* Error code signifies that the sequence is not yet done. */
				return 0;	
			}
			else
			{
				/* Error; reset RX state. */ 
				server_protocol_state.rx_state = SERVER_RX_STATE_COMMAND;
				return err_code;
			}
			break;
		}
		case UART_DFU_OPCODE_OFFSET:
		{
			size_t offset;
			int status;
			
			status = server->callbacks.offset_callback(&offset);

			if (status == 0)
			{
				if (offset <= OFFSET_MAX_ALLOWED)
				{
					reply.args.status.data.offset = offset;
				}
				else
				{
					/* TODO: call error callback. */
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
				server->callbacks.done_callback((bool) message->args.done.success);
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
	err_code = server_send(&reply.header, &reply.args, sizeof(struct uart_dfu_status_args));
	if (err_code <= 0)
	{
		/* TODO: call error callback. */
		LOG_ERR("server_send: error %d", err_code);
	}

	return err_code;
}

static int server_tx_handle(void)
{
	(void) atomic_set(&server_protocol_state.tx_opcode, OPCODE_NONE);
	return 0;
}

static int uart_dfu_rx_parse(u8_t * buf, size_t len)
{
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

	if (message->header.status)
	{
		return client_rx_handle(message, arg_size);
	}
	else
	{
		return server_rx_handle(message, arg_size);
	}
}

static void server_rx_params_get(u32_t * rx_size, u32_t * rx_timeout)
{
	enum server_rx_state rx_state;

	rx_state = (enum server_rx_state) atomic_get(&server_protocol_state.rx_state);

	switch (rx_state)
	{
		case SERVER_RX_STATE_WAIT:
		{
			*rx_size = RX_WAIT_SIZE;
			break;
		}
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

static void client_rx_params_get(u32_t * rx_size, u32_t * rx_timeout)
{
	if (atomic_get(&client_protocol_state.rx_opcode) != OPCODE_NONE)
	{
		*rx_size = RX_NORMAL_SIZE;
	}
	else
	{
		*rx_size = 0;
	}

	*rx_timeout = RX_TIMEOUT;
}

static int uart_dfu_rx_next(void)
{
	u32_t server_rx_size;
	u32_t server_rx_timeout;
	u32_t client_rx_size;
	u32_t client_rx_timeout;
	u32_t rx_size;
	u32_t rx_timeout;
	atomic_t rx_state_next;

	server_rx_params_get(&server_rx_size, &server_rx_timeout);
	client_rx_params_get(&client_rx_size, &client_rx_timeout);

	if (server_rx_size >= client_rx_size)
	{
		rx_size = server_rx_size;
		rx_timeout = server_rx_timeout;
	}
	else
	{
		rx_size = client_rx_size;
		rx_timeout = client_rx_timeout;
	}
	
	if (rx_size == 0)
	{
		return 0;
	}

	if (cobs_decode_in_frame(&rx_data_decoder))
	{
		if (cobs_decode_current_size(&rx_data_decoder) < rx_size)
		{
			rx_size -= cobs_decode_current_size(&rx_data_decoder);
		}
		else
		{
			rx_size = RX_WAIT_SIZE;
		}
	}

	rx_state_next = rx_size == RX_WAIT_SIZE ? RX_STATE_WAIT : RX_STATE_ACTIVE;
	if (atomic_cas(&protocol_state.rx_state, (atomic_t) RX_STATE_STOPPED, rx_state_next))
	{
		LOG_DBG("Enabling RX (size=%u, timeout=%d)", rx_size, rx_timeout);
		return uart_rx_enable(uart_dev, rx_data, rx_size, rx_timeout);
	}
	else
	{
		return -EBUSY;
	}
}

static int uart_dfu_rx_start(void)
{
	if (atomic_get(&protocol_state.rx_state) == (atomic_t) RX_STATE_STOPPED)
	{
		return uart_dfu_rx_next();
	}
	else
	{
		return 0;
	}
}

static int client_transaction_reserve(u8_t opcode)
{
	/* Reserve TX. */
	if (!atomic_cas(&client_protocol_state.tx_state,
					(atomic_t) CLIENT_TX_STATE_STOPPED,
					(atomic_t) CLIENT_TX_STATE_COMMAND))
	{
		return -EBUSY;
	}

	/* Reserve RX. */
	if (!atomic_cas(&client_protocol_state.rx_opcode,
				    OPCODE_NONE,
					(atomic_t) opcode))
	{
		atomic_set(&client_protocol_state.tx_state, (atomic_t) CLIENT_TX_STATE_STOPPED);
		return -EBUSY;
	}

	return 0;
}

static void client_transaction_cancel(void)
{
	/* Reset TX/RX state. */
	atomic_set(&client_protocol_state.rx_opcode, OPCODE_NONE);			
	atomic_set(&client_protocol_state.tx_state, (atomic_t) CLIENT_TX_STATE_STOPPED);
}


/*
 * UART event handlers 
 *****************************************************************************/

static void tx_buf_process(struct k_work * work)
{
	int err;
	struct tx_buf_info * info;

	info = CONTAINER_OF(work, struct tx_buf_info, work);
	if (info->buf == tx_data_client)
	{
		err = client_tx_handle();
	}
	else
	{
		err = server_tx_handle();
	}

	if (err != 0)
	{
		LOG_ERR("Error handling TX: %d", err);
	}
}

static void tx_avail_process(struct k_work * work)
{
	int err;
	
	ARG_UNUSED(work);
	
	/* XXX: Is there a problem in updating this state here? */
	atomic_set(&protocol_state.tx_state, (atomic_t) TX_STATE_STOPPED);
	
	err = uart_dfu_send_ready();
	if (err < 0)
	{
		/* TODO: call error callback. */
		LOG_ERR("Error %d while trying to send data.", err);
	}
}

static void uart_tx_done_handle(struct uart_event_tx * evt)
{
	LOG_DBG("TX (len=%u)", evt->len);

	tx_buf_info.buf = evt->buf;
	tx_buf_info.len = evt->len;

	k_work_submit(&tx_buf_info.work);
	k_work_submit(&tx_avail_work);
}

static void uart_tx_aborted_handle(struct uart_event_tx * evt)
{
	atomic_set(&protocol_state.tx_state, (atomic_t) TX_STATE_STOPPED);
	
	/* TODO: Possibly start processing work. Do once TX timeout is added. */
}

static void rx_buf_process(struct k_work * work)
{
	int err;
	struct rx_buf_info * info;
	size_t offset;
	size_t bound;
	size_t decoded_size;

	info = CONTAINER_OF(work, struct rx_buf_info, work);
	offset = info->offset;
	bound = offset + info->len;

	LOG_DBG("RX process (offset=%u, len=%u)", info->offset, info->len);

	while (offset < bound)
	{
		err = cobs_decode(&rx_data_decoder,
						  &decoded_size,
						  &info->buf[info->offset],
						  &offset,
						  bound - offset);
		if (err == -EMSGSIZE)
		{
			LOG_DBG("Message not yet complete.");
		}
		if (err == 0)
		{
			err = uart_dfu_rx_parse(rx_data_decoded, decoded_size);
			if (err >= 0)
			{
				LOG_DBG("Successfully parsed PDU.");
			}
			else
			{
				LOG_ERR("Error %d while parsing PDU.", err);
				LOG_HEXDUMP_ERR(&info->buf[info->offset], info->len, "Encoded data (full):");
				LOG_HEXDUMP_ERR(rx_data_decoded, decoded_size, "Decoded data:");
			}
		}
		else	
		{
			/* TODO: call error callback. */
			LOG_ERR("Error %d while decoding data.", err);
		}
	}

	LOG_DBG("Finished processing received data.");
}

static void rx_avail_process(struct k_work * work)
{
	int err;

	ARG_UNUSED(work);

	/* XXX: Is there a problem in updating this state here? */
	atomic_set(&protocol_state.rx_state, (atomic_t) RX_STATE_STOPPED);
	
	/* Resume reception if applicable. */
	err = uart_dfu_rx_next();
	if (err != 0 && err != -EINVAL)
	{
		/* TODO: call error callback */
		LOG_ERR("Error %d while starting RX.", err);
		k_oops();
	}
}

static void uart_rx_rdy_handle(struct uart_event_rx * evt)
{
	LOG_DBG("RX (offset=%u, len=%u)", evt->offset, evt->len);

	rx_buf_info.buf = evt->buf;
	rx_buf_info.offset = evt->offset;
	rx_buf_info.len = evt->len;

	k_work_submit(&rx_buf_info.work);
}

static void uart_rx_buf_request_handle(void)
{
	/* Double buffering is not used for now, so no implementation is needed. */
}

static void uart_rx_buf_released_handle(struct uart_event_rx_buf * evt)
{
	/* Double buffering is not used for now, so no implementation is needed. */
}

static void uart_rx_disabled_handle(void)
{
	k_work_submit(&rx_avail_work);
}

static void uart_rx_stopped_handle(struct uart_event_rx_stop * evt)
{
	/* TODO */
}

static void uart_async_cb(struct uart_event * evt, void * user_data)
{
	ARG_UNUSED(user_data);

	switch (evt->type)
	{
		case UART_TX_DONE:
		{
			LOG_DBG("UART_TX_DONE");
			uart_tx_done_handle(&evt->data.tx);
			break;
		}
		case UART_TX_ABORTED:
		{
			LOG_DBG("UART_TX_ABORTED");
			uart_tx_aborted_handle(&evt->data.tx);
			break;
		}
		case UART_RX_RDY:
		{
			LOG_DBG("UART_RX_RDY");
			uart_rx_rdy_handle(&evt->data.rx);
			break;
		}
		case UART_RX_BUF_REQUEST:
		{
			LOG_DBG("UART_RX_BUF_REQUEST");
			uart_rx_buf_request_handle();
			break;
		}
		case UART_RX_BUF_RELEASED:
		{
			LOG_DBG("UART_RX_BUF_RELEASED");
			uart_rx_buf_released_handle(&evt->data.rx_buf);
			break;
		}
		case UART_RX_DISABLED:
		{
			LOG_DBG("UART_RX_DISABLED");
			uart_rx_disabled_handle();
			break;
		}
		case UART_RX_STOPPED:
		{
			LOG_DBG("UART_RX_STOPPED");
			uart_rx_stopped_handle(&evt->data.rx_stop);
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

static int uart_dfu_sys_init(struct device * dev)
{
	int err_code;

	ARG_UNUSED(dev);

	printk("UART DFU initializing with %s.\n", CONFIG_UART_DFU_UART_LABEL);

	/* FIXME: DT_LABEL(DT_NODELABEL(uartn)) ? */

	uart_dev = device_get_binding(CONFIG_UART_DFU_UART_LABEL);
	if (uart_dev == NULL)
	{
		return -EACCES;
	}

	printk("UART DFU setting callback.\n");
	err_code = uart_callback_set(uart_dev, uart_async_cb, NULL);
	if (err_code != 0)
	{
		uart_dev = NULL;
		return -EIO;
	}

	/* TODO: Module structures init */
	memset(&protocol_state, 0, sizeof(struct module_protocol_state));
	(void) cobs_decode_reset(&rx_data_decoder);
	client = NULL;
	server = NULL;
	
	k_work_init(&rx_buf_info.work, rx_buf_process);
	k_work_init(&rx_avail_work, rx_avail_process);
	k_work_init(&tx_buf_info.work, tx_buf_process);
	k_work_init(&tx_avail_work, tx_avail_process);

	printk("UART DFU module initialized.\n");

	return 0;
}

int uart_dfu_init(void)
{
	return uart_dfu_sys_init(NULL);
}

int uart_dfu_uninit(void)
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

int uart_dfu_client_set(struct uart_dfu_client * dfu_client,
						struct uart_dfu_client_callbacks * callbacks)
{
	if (dfu_client == NULL || callbacks == NULL)
	{
		return -EINVAL; 
	}

	memset(dfu_client, 0, sizeof(struct uart_dfu_client));
	dfu_client->callbacks = *callbacks;

	memset(&client_protocol_state, 0, sizeof(struct client_protocol_state));
	atomic_set(&client_protocol_state.rx_opcode, OPCODE_NONE);

	client = dfu_client;

	return 0;
}


int uart_dfu_client_init_send(size_t file_size)
{
	int err_code;
	struct uart_dfu_message message;

	if (uart_dev == NULL || client == NULL)
	{
		return -EACCES;
	}

	if (file_size > INT32_MAX)
	{
		return -ENOMEM;
	}

	err_code = client_transaction_reserve(UART_DFU_OPCODE_INIT);
	if (err_code != 0)
	{
		return err_code;
	}

	memset(&message, 0, sizeof(struct uart_dfu_message));
	message.header.opcode = UART_DFU_OPCODE_INIT;
	message.args.init.file_size = (u32_t) file_size;

	err_code = client_send(&message.header, &message.args, sizeof(struct uart_dfu_init_args));

	if (err_code >= 0 || err_code == -EEXIST)
	{
		/* Start RX if it is not already running. */
		err_code = uart_dfu_rx_start();
	}
	if (err_code != 0)	
	{
		client_send_cancel();
		client_transaction_cancel();
	}
	
	return err_code;
}

int uart_dfu_client_write_send(const u8_t * const fragment_buf,
							   size_t fragment_size)
{
	int err_code;
	struct uart_dfu_message message;

	if (uart_dev == NULL || client == NULL)
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

	err_code = client_transaction_reserve(UART_DFU_OPCODE_WRITEH);
	if (err_code != 0)
	{
		return err_code;
	}

	client->fragment_buf = fragment_buf;
	client->fragment_idx = 0;
	client->fragment_total_size = fragment_size;
	
	memset(&message, 0, sizeof(struct uart_dfu_message));
	message.header.opcode = UART_DFU_OPCODE_WRITEH;
	message.args.writeh.fragment_total_size = fragment_size;

	err_code = client_send(&message.header, &message.args, sizeof(struct uart_dfu_writeh_args));

	if (err_code >= 0 || err_code == -EEXIST)
	{
		/* Start RX if it is not already running. */
		err_code = uart_dfu_rx_start();
	}
	if (err_code != 0)	
	{
		client_send_cancel();
		client_transaction_cancel();
	}
	
	return err_code;
}

int uart_dfu_client_offset_send(void)
{
	int err_code;
	struct uart_dfu_message message;

	if (uart_dev == NULL || client == NULL)
	{
		return -EACCES;
	}

	err_code = client_transaction_reserve(UART_DFU_OPCODE_OFFSET);
	if (err_code != 0)
	{
		return err_code;
	}

	memset(&message, 0, sizeof(struct uart_dfu_message));
	message.header.opcode = UART_DFU_OPCODE_OFFSET;

	err_code = client_send(&message.header, &message.args, sizeof(struct uart_dfu_offset_args));

	if (err_code >= 0 || err_code == -EEXIST)
	{
		/* Start RX if it is not already running. */
		err_code = uart_dfu_rx_start();
	}
	if (err_code != 0)	
	{
		client_send_cancel();
		client_transaction_cancel();
	}
	
	return err_code;
}

int uart_dfu_client_done_send(bool successful)
{
	int err_code;
	struct uart_dfu_message message;

	if (uart_dev == NULL || client == NULL)
	{
		return -EACCES;
	}

	err_code = client_transaction_reserve(UART_DFU_OPCODE_DONE);
	if (err_code != 0)
	{
		return err_code;
	}

	memset(&message, 0, sizeof(struct uart_dfu_message));
	message.header.opcode = UART_DFU_OPCODE_DONE;
	message.args.done.success = successful;

	err_code = client_send(&message.header, &message.args, sizeof(struct uart_dfu_done_args));

	if (err_code >= 0 || err_code == -EEXIST)
	{
		/* Start RX if it is not already running. */
		err_code = uart_dfu_rx_start();
	}
	if (err_code != 0)	
	{
		client_send_cancel();
		client_transaction_cancel();
	}
	
	return err_code;
}

int uart_dfu_client_stop(void)
{
	return -ENOTSUP;
}

int uart_dfu_server_set(struct uart_dfu_server * dfu_server,
						u8_t * fragment_buf,
						size_t fragment_max_size,
						struct uart_dfu_server_callbacks * callbacks)
{
	if (dfu_server == NULL 		||
		fragment_buf == NULL 	||
		fragment_max_size == 0 	||
		callbacks == NULL)
	{
		return -EINVAL;
	}
	
	memset(dfu_server, 0, sizeof(struct uart_dfu_server));
	dfu_server->fragment_buf = fragment_buf;
	dfu_server->fragment_max_size = fragment_max_size;
	dfu_server->callbacks = *callbacks;

	memset(&server_protocol_state, 0, sizeof(struct server_protocol_state));
	atomic_set(&server_protocol_state.tx_opcode, OPCODE_NONE);

	server = dfu_server;

	return 0;
}

int uart_dfu_server_enable(void)
{
	int err_code;

	if (uart_dev == NULL || server == NULL)
	{
		return -EACCES;
	}

	if (atomic_cas(&server_protocol_state.rx_state,
				   (atomic_t) SERVER_RX_STATE_STOPPED,
				   (atomic_t) SERVER_RX_STATE_WAIT))
	{
		err_code = uart_dfu_rx_start();
		if (err_code != 0)
		{
			atomic_set(&server_protocol_state.rx_state,
					   (atomic_t) SERVER_RX_STATE_STOPPED);
		}
		return err_code;
	}
	else
	{
		/* Already started. */
		return 0;
	}
}

int uart_dfu_server_disable(void)
{
	if (uart_dev == NULL || server == NULL)
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
// SYS_INIT(uart_dfu_sys_init, APPLICATION, CONFIG_UART_DFU_INIT_PRIORITY);
#endif