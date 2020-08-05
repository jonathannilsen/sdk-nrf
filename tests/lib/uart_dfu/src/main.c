/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#include <string.h>
#include <zephyr/types.h>
#include <stdbool.h>
#include <ztest.h>
#include <drivers/uart.h>
#include <cobs.h>
#include <uart_dfu_types.h>
#include <uart_dfu.h>
#include <math.h>

/* TODO: rewrite to avoid complex driver behaviour. */

/*****************************************************************************
 * Macros 
 *****************************************************************************/

#define UART_DEVICE_COUNT 	1

#define INT_COMPARE_STRING			" Got %d (expected %d)."
#define UINT_COMPARE_STRING			" Got %u (expected %u)."

#define ASSERT_INT_EQUAL(a, b, msg)	zassert_equal(a, b, msg INT_COMPARE_STRING, a, b)
#define ASSERT_UINT_EQUAL(a, b, msg) zassert_equal(a, b, msg UINT_COMPARE_STRING, a, b) 

#define ERROR_MSG_STRING 			"Unexpected error code: %d."
#define ERROR_CHECK(err, exp_err)	zassert_equal(err, exp_err, ERROR_MSG_STRING, err)

#define SERVER_FRAGMENT_BUF_SIZE	1024

/*****************************************************************************
 * Structure definitions 
 *****************************************************************************/

struct uart_device_info {
	uart_callback_t callback;
	void * user_data;

	const u8_t * tx_data;
	size_t tx_len;
	size_t tx_timeout;

	u8_t * rx_data;
	size_t rx_offset;
	size_t rx_len;
	size_t rx_timeout;
	bool rx_timed_out;
};

struct client_callback_record { 
	int status_called;
	int last_status;
	
	int offset_called;
	size_t last_offset;
	
	int available_called;
	
	int error_called;
	int last_error;
};

struct server_callback_record {
	int init_called;
	size_t last_file_size;
	int init_return_code;

	int write_called;
	u8_t * last_fragment_buf;
	size_t last_fragment_size;
	int write_return_code;
	
	int offset_called;
	size_t offset_return_value;
	int offset_return_code;

	int done_called;
	bool last_successful_value;
	int done_return_code;

	int error_called;
	int last_error;
};


/*****************************************************************************
 * Static variables 
 *****************************************************************************/

static struct device uart_device;
static struct uart_device_info uart_device_info;

static struct client_callback_record client_record;
static struct uart_dfu_client client;

static struct server_callback_record server_record;
static struct uart_dfu_server server;
static u8_t fragment_buffer[SERVER_FRAGMENT_BUF_SIZE];

static u8_t tx_buffer[COBS_MAX_BYTES];
static size_t tx_size;


/*****************************************************************************
 * Mock UART implementation 
 *****************************************************************************/

struct device * device_get_binding_stub(const char * name)
{
	zassert_not_null(name, "UART label is NULL");
	return &uart_device;
}

int uart_callback_set_stub(struct device *dev,
				    	   uart_callback_t callback,
				    	   void *user_data)
{
	zassert_equal_ptr(dev, &uart_device, "Invalid device pointer");
	zassert_not_null(callback, "Callback is NULL");

	uart_device_info.callback = callback;
	uart_device_info.user_data = user_data;

	return 0;
}

int uart_tx_stub(struct device *dev,
				 const u8_t *buf,
				 size_t len,
				 s32_t timeout)
{
	zassert_equal_ptr(dev, &uart_device, "Invalid device pointer");
	zassert_not_null(buf, "UART TX buffer is NULL");
	zassert_not_equal(len, 0, "UART TX length of 0.");

	zassert_equal(uart_device_info.tx_data, NULL, "UART TX before previous TX done.");

	uart_device_info.tx_data = buf;
	uart_device_info.tx_len = len;
	uart_device_info.tx_timeout = timeout;

	return 0;	
}

int uart_rx_enable_stub(struct device *dev,
						u8_t *buf,
						size_t len,
						s32_t timeout)
{
	zassert_equal_ptr(dev, &uart_device, "Invalid device pointer");
	zassert_not_null(buf, "UART RX buffer is NULL");
	zassert_not_equal(len, 0, "UART RX length of 0.");

	zassert_equal(uart_device_info.rx_data, NULL, "UART RX before previous RX done.");

	uart_device_info.rx_data = buf;
	uart_device_info.rx_offset = 0;
	uart_device_info.rx_len = len;
	uart_device_info.rx_timeout = timeout;

	return 0;
}

int uart_rx_disable_stub(struct device *dev)
{
	zassert_equal_ptr(dev, &uart_device, "Invalid device pointer");

	zassert_equal(uart_device_info.rx_data, NULL, "UART RX disable without ongoing RX.");

	/* TODO */

	return 0;
}

int uart_rx_buf_rsp_stub(struct device *dev, u8_t *buf, size_t len)
{
	zassert_true(false, "UART RX double buffer not supported.");
	return 0;
}

static void tx_data_send(size_t len)
{
	struct uart_event tx_event;

	zassert_not_null(uart_device_info.tx_data, "TX buffer pointer should not be NULL");
	zassert_true(0 < uart_device_info.tx_len && uart_device_info.tx_len <= sizeof(tx_buffer),
				 "TX data length should be valid. Was: %u.",
				 uart_device_info.tx_len);

	/* Save sent data for inspection. */
	memcpy(tx_buffer, uart_device_info.tx_data, uart_device_info.tx_len);
	tx_size = uart_device_info.tx_len;

	tx_event.type = UART_TX_DONE;
	tx_event.data.tx.buf = uart_device_info.tx_data;
	tx_event.data.tx.len = MIN(uart_device_info.tx_len, len);

	uart_device_info.tx_data = NULL;
	uart_device_info.tx_len = 0;
	uart_device_info.tx_timeout = SYS_FOREVER_MS;

	/* Send TX event to sender */
	uart_device_info.callback(&tx_event, uart_device_info.user_data);
}

static void tx_data_send_all(void)
{
	tx_data_send(uart_device_info.tx_len);
}

static int rx_data_send(u8_t * buf, size_t len, bool timeout)
{
	struct uart_event rx_event;
	size_t remaining_size = len;

	while (remaining_size > 0 && uart_device_info.rx_data != NULL)
	{
		size_t rx_size = MIN(remaining_size,
							 uart_device_info.rx_len - uart_device_info.rx_offset);
		memcpy(uart_device_info.rx_data, &buf[len - remaining_size], rx_size);
		size_t prev_offset = uart_device_info.rx_offset;
		uart_device_info.rx_offset += rx_size;
		remaining_size -= rx_size;
		
		if (uart_device_info.rx_offset == uart_device_info.rx_len ||
			(timeout && uart_device_info.rx_timeout != SYS_FOREVER_MS))
		{
			/* Buffer filled or timeout occurred. */
			memset(&rx_event, 0, sizeof(struct uart_event));
			rx_event.type = UART_RX_RDY;
			rx_event.data.rx.buf = uart_device_info.rx_data;
			rx_event.data.rx.offset = prev_offset;
			rx_event.data.rx.len = rx_size;

			uart_device_info.callback(&rx_event, uart_device_info.user_data);

			if (uart_device_info.rx_offset == uart_device_info.rx_len)
			{
				/* Buffer was filled - generate appropriate events. */
				memset(&rx_event, 0, sizeof(struct uart_event));
				rx_event.type = UART_RX_BUF_RELEASED;
				rx_event.data.rx_buf.buf = uart_device_info.rx_data;

				/* Clear RX state before sending event. */
				uart_device_info.rx_data = NULL;
				uart_device_info.rx_offset = 0;
				uart_device_info.rx_len = 0;
				uart_device_info.rx_timeout = 0;			

				uart_device_info.callback(&rx_event, uart_device_info.user_data);

				memset(&rx_event, 0, sizeof(struct uart_event));
				rx_event.type = UART_RX_DISABLED;

				uart_device_info.callback(&rx_event, uart_device_info.user_data);
			}
		}
	}

	if (remaining_size > 0)
	{
		return -ENOMEM;
	}

	return 0;
}

static void encode_and_send(u8_t * buf, size_t len, bool timeout)
{
	int err_code;

	u8_t encoded_data[COBS_MAX_BYTES];
	size_t encoded_size;
	COBS_ENCODER_DECLARE(encoder, encoded_data);

	err_code = cobs_encode(&encoder, buf, len);
	zassert_equal(err_code, 0, ERROR_MSG_STRING, err_code);

	err_code = cobs_encode_finish(&encoder, &encoded_size);
	zassert_equal(err_code, 0, ERROR_MSG_STRING, err_code);

	err_code = rx_data_send(encoded_data, encoded_size, timeout);
	zassert_equal(err_code, 0, ERROR_MSG_STRING, err_code);
}

static struct uart_dfu_message * tx_data_decode(u8_t * decoded_data, size_t * size, size_t expected_size)
{
	int err_code;
	size_t decoded_size;
	size_t encoded_offset = 0;
	COBS_DECODER_DECLARE(decoder, decoded_data);

	if (expected_size > 0)
	{
		ASSERT_UINT_EQUAL(tx_size, expected_size, "Unexpected TX size.");
	}
	else
	{
		zassert_true(tx_size >= COBS_ENCODED_SIZE(0), "TX data should not be empty.");	
	}

	err_code = cobs_decode(&decoder, &decoded_size, tx_buffer, &encoded_offset, tx_size);
	ERROR_CHECK(err_code, 0);

	if (size != NULL)
	{
		*size = decoded_size;
	}
	return (struct uart_dfu_message *) decoded_data;
}


/*****************************************************************************
 * Test helper functions 
 *****************************************************************************/

static void uart_dfu_test_init(void)
{
	int err_code;

	memset(&uart_device, 0, sizeof(struct device));	
	memset(&uart_device_info, 0, sizeof(struct uart_device_info));
	memset(tx_buffer, 0, sizeof(tx_buffer));
	tx_size = 0;

	err_code = uart_dfu_init();
	zassert_equal(err_code, 0, ERROR_MSG_STRING, err_code);
}

static void client_status_handle(int status)
{
	client_record.status_called++;
	client_record.last_status = status;
}

static void client_offset_handle(size_t offset)
{
	client_record.offset_called++;
	client_record.last_offset = offset;
}

static void client_available_handle(void)
{
	client_record.available_called++;
}

static void client_error_handle(int error)
{
	client_record.error_called++;
	client_record.last_error = error;
}

static void client_init(void)
{
	int err_code;
	struct uart_dfu_client_callbacks callbacks;

	memset(&client_record, 0, sizeof(struct client_callback_record));
	memset(&client, 0, sizeof(struct uart_dfu_client));

	callbacks.status_callback = client_status_handle;
	callbacks.offset_callback = client_offset_handle;
	callbacks.available_callback = client_available_handle;
	callbacks.error_callback = client_error_handle;

	err_code = uart_dfu_client_set(&client, &callbacks);
	zassert_equal(err_code, 0, ERROR_MSG_STRING, err_code);
}

static int server_init_handle(size_t file_size)
{
	server_record.init_called++;
	server_record.last_file_size = file_size;
	return server_record.init_return_code;
}

static int server_write_handle(const u8_t * const fragment_buf,
							   size_t fragment_size)
{
	server_record.write_called++;
	server_record.last_fragment_buf = (u8_t *) fragment_buf;
	server_record.last_fragment_size = fragment_size;
	return server_record.write_return_code;
}

static int server_offset_handle(size_t * offset)
{
	server_record.offset_called++;
	*offset = server_record.offset_return_value;
	return server_record.offset_return_code;
}

static int server_done_handle(bool successful)
{
	server_record.done_called++;
	server_record.last_successful_value = successful;
	return server_record.done_return_code;
}

static void server_error_handle(int error)
{
	server_record.error_called++;
	server_record.last_error = error;
}

static void server_init(void)
{
	int err_code;
	struct uart_dfu_server_callbacks callbacks;

	memset(&server, 0, sizeof(struct uart_dfu_server));
	memset(&server_record, 0, sizeof(struct server_callback_record));

	callbacks.init_callback = server_init_handle;
	callbacks.write_callback = server_write_handle;
	callbacks.offset_callback = server_offset_handle;
	callbacks.done_callback = server_done_handle;
	callbacks.error_callback = server_error_handle;

	err_code = uart_dfu_server_set(&server,
								   fragment_buffer,
								   SERVER_FRAGMENT_BUF_SIZE,
								   &callbacks);
	ERROR_CHECK(err_code, 0);
}

static inline void pdu_validate(struct uart_dfu_message * message,
								u8_t opcode,
								u8_t status,
								union uart_dfu_args * args,
								size_t arg_size)
{
	ASSERT_UINT_EQUAL(message->header.opcode, opcode, "Bad opcode.");
	ASSERT_UINT_EQUAL(message->header.status, status, "Wrong status bit.");
	zassert_mem_equal(&message->args, args, arg_size, "Unexpected arguments.");
}


/*****************************************************************************
 * Unit test functions 
 *****************************************************************************/

static void test_cobs_encode_decode_simple(void)
{
	u8_t decoded_data_expected[] = {
		0x01, 0x02, 0x03
	};
	size_t decoded_size_expected = sizeof(decoded_data_expected);
	
	int err_code;
	
	/* Test encoding */
	u8_t encoded_data[COBS_MAX_BYTES];
	size_t encoded_size;
	COBS_ENCODER_DECLARE(encoder, encoded_data);	
	
	err_code = cobs_encode(&encoder, decoded_data_expected, decoded_size_expected);
	zassert_equal(err_code, 0, ERROR_MSG_STRING, err_code);
	err_code = cobs_encode_finish(&encoder, &encoded_size);
	zassert_equal(encoded_size,
				  decoded_size_expected + COBS_OVERHEAD_BYTES,
				  "COBS encoding should add a constant overhead.");
	zassert_equal(encoded_data[0],
				  encoded_size - 1,
				  "Overhead byte should point to the delimiter if there are no other zeroes in the frame.");
	zassert_equal(encoded_data[encoded_size - 1],
				  COBS_DELIMITER,
				  "COBS should add a delimiter at the end of the frame");
	zassert_mem_equal(&encoded_data[1],
					  &decoded_data_expected[0],
					  decoded_size_expected,
					  "COBS should not modify frame contents if there are no zeroes.");

	/* Test decoding */
	u8_t decoded_data[COBS_MAX_BYTES];
	size_t decoded_size;
	size_t encoded_offset;
	COBS_DECODER_DECLARE(decoder, decoded_data);

	memset(decoded_data, 0xFF, sizeof(decoded_data));
	decoded_size = 0;
	encoded_offset = 0;	
	err_code = cobs_decode(&decoder,
						   &decoded_size,
						   encoded_data,
						   &encoded_offset,
						   encoded_size);
	zassert_equal(err_code, 0, ERROR_MSG_STRING, err_code);
	zassert_equal(encoded_offset,
				  encoded_size,
				  "COBS decoding should return correct offset of consumed bytes.");
	zassert_equal(decoded_size,
				  sizeof(decoded_data_expected),
				  "COBS decoding should result in the original length.");
	zassert_mem_equal(decoded_data,
					  decoded_data_expected,
					  decoded_size,
					  "COBS decoding should return the original value.");
}

static void test_uart_dfu_client_init_send(void)
{
	int err_code;

	uart_dfu_test_init();
	client_init();

	size_t file_size = 1024;
	err_code = uart_dfu_client_init_send(file_size);
	ERROR_CHECK(err_code, 0);

	tx_data_send_all();
	/* XXX: Validate sent data? */

	struct uart_dfu_message message;
	memset(&message, 0, sizeof(struct uart_dfu_message));
	message.header.opcode = UART_DFU_OPCODE_INIT;
	message.header.status = 1;
	message.args.status.data.status = 0;
	encode_and_send((u8_t *) &message, sizeof(struct uart_dfu_message), true);

	ASSERT_INT_EQUAL(client_record.status_called, 1, "status_callback should be called.");
	ASSERT_INT_EQUAL(client_record.last_status, 0, "last_status should be 0.");
}

static void test_uart_dfu_client_write_send(void)
{
	int err_code;

	uart_dfu_test_init();
	client_init();

	u8_t fragment_buf[512];
	size_t fragment_size = sizeof(fragment_buf);

	/* Fill the fragment with data that makes it easier to recognize. */
	u8_t byte = 0x02;
	for (size_t i = 0; i < fragment_size; i++)
	{
		fragment_buf[i] = byte;
		byte = byte == UINT8_MAX ? 0x02 : byte + 1; 
	}

	err_code = uart_dfu_client_write_send(fragment_buf, fragment_size);
	ERROR_CHECK(err_code, 0);

	size_t segment_count;
	u8_t decoded_data[COBS_MAX_DATA_BYTES];
	size_t decoded_size;
	size_t encoded_offset = 0;
	COBS_DECODER_DECLARE(decoder, decoded_data);
	struct uart_dfu_message * message;

	tx_data_send_all();
	err_code = cobs_decode(&decoder, &decoded_size, tx_buffer, &encoded_offset, tx_size);
	ERROR_CHECK(err_code, 0);
	zassert_equal(decoded_size,
				  sizeof(struct uart_dfu_message),
				  "Sent data should have correct size.");

	message = (struct uart_dfu_message *) decoded_data;
	zassert_equal(message->header.opcode, UART_DFU_OPCODE_WRITEH, "First WRITE PDU should be WRITEH");
	zassert_not_equal(message->header.status, 1, "Client message should not have status bit set.");
	zassert_equal(message->args.writeh.fragment_total_size,
				  fragment_size,
				  "WRITE should send correct fragment size.");

	struct uart_dfu_message response;
	memset(&response, 0, sizeof(struct uart_dfu_message));
	response.header.opcode = UART_DFU_OPCODE_WRITEH;
	response.header.status = 1;
	response.args.status.data.status = 0;
	encode_and_send((u8_t *) &response, sizeof(struct uart_dfu_message), true);

	u8_t fragment_received_buf[512];
	size_t fragment_received_size = 0;
	struct uart_dfu_header * header;

	for (segment_count = 0; uart_device_info.tx_data != NULL; segment_count++)
	{
		tx_data_send_all();
		zassert_true(tx_size > COBS_ENCODED_SIZE(sizeof(struct uart_dfu_header)),
					 "Segment should not be empty.");
		encoded_offset = 0;
		err_code = cobs_decode(&decoder, &decoded_size, tx_buffer, &encoded_offset, tx_size);
		ERROR_CHECK(err_code, 0);
		header = (struct uart_dfu_header *) decoded_data;
		zassert_equal(header->opcode,
					  UART_DFU_OPCODE_WRITEC,
					  "Following WRITE PDUs should be WRITEC.");
		zassert_not_equal(header->status, 1, "Client message should not have status bit set.");
		zassert_true(fragment_received_size + decoded_size - sizeof(struct uart_dfu_header) <= fragment_size,
					 "Fragment size should not exceed intended size.");
		memcpy(&fragment_received_buf[fragment_received_size],
			   &decoded_data[sizeof(struct uart_dfu_header)],
			   decoded_size - sizeof(struct uart_dfu_header));	
		fragment_received_size += decoded_size - sizeof(struct uart_dfu_header);
	}

	size_t expected_segment_count = (size_t) ceil((double) (fragment_size) / 
									(double) (COBS_MAX_DATA_BYTES - sizeof(struct uart_dfu_header)));
	zassert_equal(segment_count,
				  expected_segment_count,
				  "Wrong segment count: %u (expected: %u)",
				  segment_count,
				  expected_segment_count);
	zassert_equal(fragment_received_size, fragment_size, "Received and sent fragment size not equal.");
	zassert_mem_equal(fragment_received_buf,
					  fragment_buf,
					  fragment_size,
					  "Received and sent fragment should be equal.");
}

static void test_uart_dfu_client_offset_send(void)
{
	int err_code;

	uart_dfu_test_init();
	client_init();

	err_code = uart_dfu_client_offset_send();
	ERROR_CHECK(err_code, 0);

	tx_data_send_all();

	size_t offset_value = 512;

	struct uart_dfu_message response;
	response.header.opcode = UART_DFU_OPCODE_OFFSET;
	response.header.status = 1;
	response.args.status.data.offset = offset_value;

	encode_and_send((u8_t *) &response, sizeof(struct uart_dfu_message), true);

	zassert_equal(client_record.offset_called, 1, "offset_callback should be called.");
	zassert_equal(client_record.last_offset,
				  offset_value,
				  "Wrong offset value %u (expected %u).",
				  client_record.last_offset,
				  offset_value);
	
	err_code = uart_dfu_client_offset_send();
	ERROR_CHECK(err_code, 0);

	tx_data_send_all();

	int status_value = -100;

	response.args.status.data.status = status_value;

	encode_and_send((u8_t *) &response, sizeof(struct uart_dfu_message),  true);
	
	zassert_equal(client_record.offset_called, 1, "offset_callback should not be called.");
	zassert_equal(client_record.status_called, 1, "status_callback should be called.");
	zassert_equal(client_record.last_status,
				  status_value,
				  "Wrong status value %d (expected %d).",
				  client_record.last_status,
				  status_value);
}

static void test_uart_dfu_client_done_send(void)
{
	int err_code;

	uart_dfu_test_init();
	client_init();

	err_code = uart_dfu_client_done_send(true);
	ERROR_CHECK(err_code, 0);

	tx_data_send_all();
	
	struct uart_dfu_message response;
	response.header.opcode = UART_DFU_OPCODE_DONE;
	response.header.status = 1;
	response.args.status.data.status = 0;
	encode_and_send((u8_t *) &response, sizeof(struct uart_dfu_message), true);

	ASSERT_INT_EQUAL(client_record.status_called, 1, "status_callback should be called.");
	ASSERT_INT_EQUAL(client_record.last_status, 0, "last_status should be 0.");
}

static void test_uart_dfu_server_init_receive(void)
{
	int err_code;

	uart_dfu_test_init();
	server_init();

	err_code = uart_dfu_server_enable();
	ERROR_CHECK(err_code, 0);

	size_t file_size = 2048;

	/* Return success from callback. */
	server_record.init_return_code = 0;

	struct uart_dfu_message message;
	message.header.opcode = UART_DFU_OPCODE_INIT;
	message.args.init.file_size = (u32_t) file_size;
	encode_and_send((u8_t *) &message, sizeof(message), true);

	ASSERT_INT_EQUAL(server_record.init_called, 1, "init_callback should be called.");
	ASSERT_UINT_EQUAL(server_record.last_file_size, file_size, "file size should be correct. ");

	tx_data_send_all();
	
	u8_t decoded_data[COBS_MAX_DATA_BYTES];
	struct uart_dfu_message * response;
	union uart_dfu_args expected_args = {
		.status.data.status = server_record.init_return_code	
	};
	
	response = tx_data_decode(decoded_data, NULL, COBS_ENCODED_SIZE(sizeof(struct uart_dfu_message)));
	pdu_validate(response, UART_DFU_OPCODE_INIT, 1, &expected_args, sizeof(expected_args));
}

static void test_uart_dfu_server_write_receive(void)
{
	int err_code;

	uart_dfu_test_init();
	server_init();

	err_code = uart_dfu_server_enable();
	ERROR_CHECK(err_code, 0);

	u8_t fragment_buf[512];
	size_t fragment_size = sizeof(fragment_buf);

	/* Fill the fragment with data that makes it easier to recognize. */
	u8_t byte = 0x02;
	for (size_t i = 0; i < fragment_size; i++)
	{
		fragment_buf[i] = byte;
		byte = byte == UINT8_MAX ? 0x02 : byte + 1; 
	}

	struct uart_dfu_message message;
	memset(&message, 0, sizeof(message));
	message.header.opcode = UART_DFU_OPCODE_WRITEH;
	message.args.writeh.fragment_total_size = fragment_size;
	encode_and_send((u8_t *) &message, sizeof(message), true);

	u8_t decoded_data[COBS_MAX_DATA_BYTES];
	struct uart_dfu_message * response;
	union uart_dfu_args expected_args;

	tx_data_send_all();
	response = tx_data_decode(decoded_data, NULL, COBS_ENCODED_SIZE(sizeof(struct uart_dfu_message)));
	expected_args = (union uart_dfu_args) { .status.data.status = 0 };
	pdu_validate(response, UART_DFU_OPCODE_WRITEH, 1, &expected_args, sizeof(expected_args));

	u8_t encoded_data[COBS_MAX_BYTES];
	size_t encoded_size;
	size_t fragment_offset = 0;
	size_t segment_size;
	struct uart_dfu_header header;
	COBS_ENCODER_DECLARE(encoder, encoded_data);

	/* Set write return code. */
	server_record.write_return_code = 0;

	while (fragment_offset < fragment_size)
	{
		memset(&header, 0, sizeof(header));
		header.opcode = UART_DFU_OPCODE_WRITEC;
		header.status = 0;
		
		err_code = cobs_encode(&encoder, (u8_t *) &header, sizeof(header));
		ERROR_CHECK(err_code, 0);

		segment_size = MIN(fragment_size - fragment_offset, COBS_MAX_DATA_BYTES - sizeof(header));
		err_code = cobs_encode(&encoder, &fragment_buf[fragment_offset], segment_size);
		ERROR_CHECK(err_code, 0);
		
		err_code = cobs_encode_finish(&encoder, &encoded_size);
		ERROR_CHECK(err_code, 0);

		fragment_offset += segment_size;

		err_code = rx_data_send(encoded_data, encoded_size, true);
		ERROR_CHECK(err_code, 0);	
	}

	ASSERT_INT_EQUAL(server_record.write_called, 1, "write_callback should be called.");
	ASSERT_UINT_EQUAL(server_record.last_fragment_size, fragment_size, "Incorrect fragment size.");

	tx_data_send_all();
	response = tx_data_decode(decoded_data, NULL, COBS_ENCODED_SIZE(sizeof(struct uart_dfu_message)));
	expected_args = (union uart_dfu_args) { .status.data.status = 0 };
	pdu_validate(response, UART_DFU_OPCODE_WRITEC, 1, &expected_args, sizeof(expected_args));
}

static void test_uart_dfu_server_offset_receive(void)
{
	int err_code;

	uart_dfu_test_init();
	server_init();

	err_code = uart_dfu_server_enable();
	ERROR_CHECK(err_code, 0);

	size_t offset = 123;
	int status = 0;
	server_record.offset_return_value = offset;
	server_record.offset_return_code = status;

	struct uart_dfu_message message;
	memset(&message, 0, sizeof(struct uart_dfu_message));
	message.header.opcode = UART_DFU_OPCODE_OFFSET;
	encode_and_send((u8_t *) &message, sizeof(message), true);

	ASSERT_INT_EQUAL(server_record.offset_called, 1, "offset_callback should be called.");
	
	tx_data_send_all();
	ASSERT_UINT_EQUAL(tx_size,
					  COBS_ENCODED_SIZE(sizeof(struct uart_dfu_message)),
					  "Server should respond.");

	u8_t decoded_data[COBS_MAX_BYTES];
	size_t decoded_size;
	size_t encoded_offset = 0;
	COBS_DECODER_DECLARE(decoder, decoded_data);
	struct uart_dfu_message * response;

	err_code = cobs_decode(&decoder, &decoded_size, tx_buffer, &encoded_offset, tx_size);
	ERROR_CHECK(err_code, 0);

	response = (struct uart_dfu_message *) decoded_data;
	ASSERT_UINT_EQUAL(response->header.opcode, UART_DFU_OPCODE_OFFSET, "Opcode incorrect.");
	ASSERT_UINT_EQUAL(response->header.status, 1, "Status not set.");
	ASSERT_UINT_EQUAL(response->args.status.data.offset, offset, "Offset incorrect");
}

static void test_uart_dfu_server_done_receive(void)
{
	int err_code;

	uart_dfu_test_init();
	server_init();

	err_code = uart_dfu_server_enable();
	ERROR_CHECK(err_code, 0);

	/* Set return parameters. */
	int status = 0;
	server_record.done_return_code = status;

	struct uart_dfu_message message;
	memset(&message, 0, sizeof(message));
	message.header.opcode = UART_DFU_OPCODE_DONE;
	message.args.done.success = 1;
	encode_and_send((u8_t *) &message, sizeof(message), true);

	ASSERT_INT_EQUAL(server_record.done_called, 1, "done_callback should be called.");

	tx_data_send_all();
	ASSERT_UINT_EQUAL(tx_size,
					  COBS_ENCODED_SIZE(sizeof(struct uart_dfu_message)),
					  "Server should respond.");

	u8_t decoded_data[COBS_MAX_BYTES];
	size_t decoded_size;
	size_t encoded_offset = 0;
	COBS_DECODER_DECLARE(decoder, decoded_data);
	struct uart_dfu_message * response;

	err_code = cobs_decode(&decoder, &decoded_size, tx_buffer, &encoded_offset, tx_size);
	ERROR_CHECK(err_code, 0);

	response = (struct uart_dfu_message *) decoded_data;
	ASSERT_UINT_EQUAL(response->header.opcode, UART_DFU_OPCODE_DONE, "Opcode incorrect.");
	ASSERT_UINT_EQUAL(response->header.status, 1, "Status not set.");
	ASSERT_UINT_EQUAL(response->args.status.data.status, status, "Status incorrect");
}


void test_main(void)
{
	ztest_test_suite(lib_uart_dfu_test,
		ztest_unit_test(test_cobs_encode_decode_simple),
		ztest_unit_test(test_uart_dfu_client_init_send),
		ztest_unit_test(test_uart_dfu_client_write_send),
		ztest_unit_test(test_uart_dfu_client_offset_send),
		ztest_unit_test(test_uart_dfu_client_done_send),
		ztest_unit_test(test_uart_dfu_server_init_receive),
		ztest_unit_test(test_uart_dfu_server_write_receive),
		ztest_unit_test(test_uart_dfu_server_offset_receive),
		ztest_unit_test(test_uart_dfu_server_done_receive)
	);

	ztest_run_test_suite(lib_uart_dfu_test);
}
