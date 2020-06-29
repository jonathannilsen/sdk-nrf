/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#include <string.h>
#include <zephyr/types.h>
#include <stdbool.h>
#include <ztest.h>
#include <dfu_target.h>
#include <dfu_target_uart.h>
#include <uart_dfu_cli.h>


#define TEST_FILE_SIZE 		(150 * 1024)

#define UART_DFU_HEADER_LENGTH	4
#define QUEUE_PRIORITY		(-1)
#define QUEUE_STACK_SIZE	512


enum work_flag {
	FLAG_EVT,
	FLAG_STATUS,
	FLAG_OFFSET
};


K_THREAD_STACK_DEFINE(stack_area, QUEUE_STACK_SIZE);
struct k_work_q work_queue;

static struct uart_dfu_cli_cb cli_cb;

static int status_ret;
static size_t offset_ret;
static struct uart_dfu_cli_evt evt_ret;
static atomic_t work_flags;
static struct k_work status_work;
static struct k_work offset_work;


static void async_status(struct k_work *work)
{
	ARG_UNUSED(work);

	if (atomic_test_and_clear_bit(&work_flags, FLAG_EVT)) {
		cli_cb.evt_cb(&evt_ret);
	} else if (atomic_test_and_clear_bit(&work_flags, FLAG_STATUS)) {
		cli_cb.status_cb(status_ret);
	} else {
		zassert_unreachable("No event to dispatch.");
	}
}

static void async_offset(struct k_work *work)
{
	ARG_UNUSED(work);

	if (atomic_test_and_clear_bit(&work_flags, FLAG_EVT)) {
		cli_cb.evt_cb(&evt_ret);
	} else if (atomic_test_and_clear_bit(&work_flags, FLAG_STATUS)) {
		cli_cb.status_cb(status_ret);
	} else if (atomic_test_and_clear_bit(&work_flags, FLAG_OFFSET)) {
		cli_cb.offset_cb(offset_ret);
	} else {
		zassert_unreachable("No event to dispatch.");
	}
}

static void status_post(int status)
{
	status_ret = status;
	atomic_set_bit(&work_flags, FLAG_STATUS);
}

static void offset_post(size_t offset)
{
	offset_ret = offset;
	atomic_set_bit(&work_flags, FLAG_OFFSET);
}

static void evt_post(enum uart_dfu_cli_evt_type type, enum uart_dfu_cli_err err)
{
	evt_ret.type = type;
	evt_ret.err = err;
	atomic_set_bit(&work_flags, FLAG_EVT);
}

int uart_dfu_cli_init(struct uart_dfu_cli_cb *callbacks)
{
	zassert_not_null(callbacks, NULL);
	zassert_not_null(callbacks->offset_cb, NULL);
	zassert_not_null(callbacks->status_cb, NULL);
	zassert_not_null(callbacks->evt_cb, NULL);

	cli_cb = *callbacks;
	return ztest_get_return_value();
}

int uart_dfu_cli_init_send(size_t file_size)
{
	ztest_check_expected_value(file_size);
	int err = ztest_get_return_value();
	if (err == 0) {
		k_work_submit_to_queue(&work_queue, &status_work);
		return 0;
	}
	return err;
}

int uart_dfu_cli_write_send(const uint8_t *const buf, size_t len)
{
	ztest_check_expected_value(buf);
	ztest_check_expected_value(len);
	int err = ztest_get_return_value();
	if (err == 0) {
		k_work_submit_to_queue(&work_queue, &status_work);
		return 0;
	}
	return err;
}

int uart_dfu_cli_offset_send(void)
{
	int err = ztest_get_return_value();
	if (err == 0) {
		k_work_submit_to_queue(&work_queue, &offset_work);
		return 0;
	}
	return err;
}

int uart_dfu_cli_done_send(bool successful)
{
	ztest_check_expected_value(successful);
	int err = ztest_get_return_value();
	if (err == 0) {
		k_work_submit_to_queue(&work_queue, &status_work);
		return 0;
	}
	return err;
}

int uart_dfu_cli_start(void)
{
	int err = ztest_get_return_value();
	if (err == -EINPROGRESS) {
		evt_post(UART_DFU_CLI_EVT_STARTED, UART_DFU_CLI_SUCCESS);
		k_work_submit_to_queue(&work_queue, &status_work);
	}
	return err;
}

int uart_dfu_cli_stop(void)
{
	int err = ztest_get_return_value();
	if (err == -EINPROGRESS) {
		evt_post(UART_DFU_CLI_EVT_STOPPED, UART_DFU_CLI_SUCCESS);
		k_work_submit_to_queue(&work_queue, &status_work);
	}
	return err;
}

static void dfu_target_callback(enum dfu_target_evt_id evt_id)
{
	ztest_check_expected_value(evt_id);
}

static void session_init(bool first)
{
	int err;
	size_t file_size = TEST_FILE_SIZE;
	size_t file_size_exp = file_size - UART_DFU_HEADER_LENGTH;

	(void) atomic_clear(&work_flags);
	if (first) {
		memset(&cli_cb, 0, sizeof(cli_cb));
		ztest_returns_value(uart_dfu_cli_init, 0);
	}
	ztest_returns_value(uart_dfu_cli_start, -EINPROGRESS);
	ztest_expect_value(uart_dfu_cli_init_send, file_size, file_size_exp);
	ztest_returns_value(uart_dfu_cli_init_send, 0);
	status_post(0);
	err = dfu_target_uart_init(file_size, dfu_target_callback);
	zassert_equal(err, 0, "Invalid init value: %d", err);
}

static void session_done(void)
{
	ztest_expect_value(uart_dfu_cli_done_send, successful, true);
	ztest_returns_value(uart_dfu_cli_done_send, 0);
	ztest_returns_value(uart_dfu_cli_stop, 0);
	status_post(0);
	int err = dfu_target_uart_done(true);
	zassert_equal(err, 0, "Done error: %d", err);
}

static void test_init(void)
{
	int err;
	size_t file_size = TEST_FILE_SIZE;
	size_t file_size_exp = file_size - UART_DFU_HEADER_LENGTH;

	session_init(true);

	/* Test API error */
	ztest_returns_value(uart_dfu_cli_start, 0);
	ztest_expect_value(uart_dfu_cli_init_send, file_size, file_size_exp);
	ztest_returns_value(uart_dfu_cli_init_send, -EINVAL);
	err = dfu_target_uart_init(file_size, dfu_target_callback);
	zassert_equal(err, -EINVAL, "Invalid init value: %d", err);

	/* Test protocol error */
	ztest_returns_value(uart_dfu_cli_start, 0);
	ztest_expect_value(uart_dfu_cli_init_send, file_size, file_size_exp);
	ztest_returns_value(uart_dfu_cli_init_send, 0);
	evt_post(UART_DFU_CLI_EVT_STOPPED, UART_DFU_CLI_ERR_TIMEOUT);
	err = dfu_target_uart_init(file_size, dfu_target_callback);
	zassert_equal(err, -ETIMEDOUT, "Invalid init value: %d", err);

	session_done();
}

static void test_offset_get(void)
{
	int err;
	size_t offset = 0;
	size_t offset_exp = TEST_FILE_SIZE / 2;

	session_init(false);

	/* Test offset */
	ztest_returns_value(uart_dfu_cli_offset_send, 0);
	offset_post(offset_exp);
	err = dfu_target_uart_offset_get(&offset);
	zassert_equal(err, 0, "Error: %d", err);
	zassert_equal(offset, offset_exp, "Unexpected offset");

	/* Test API error */
	ztest_returns_value(uart_dfu_cli_offset_send, -EINVAL);
	err = dfu_target_uart_offset_get(&offset);
	zassert_equal(err, -EINVAL, "Unexpected error %d", err);

	/* Test protocol error */
	ztest_returns_value(uart_dfu_cli_offset_send, 0);
	evt_post(UART_DFU_CLI_EVT_STOPPED, UART_DFU_CLI_ERR_TIMEOUT);
	err = dfu_target_uart_offset_get(&offset);
	zassert_equal(err, -ETIMEDOUT, "Unexpected error: %d", err);

	session_done();
}

static void test_write(void)
{
	uint8_t buf[1024];
	size_t len = sizeof(buf);

	session_init(false);

	/* Test short write with no data */
	int err = dfu_target_uart_write(buf, UART_DFU_HEADER_LENGTH);
	zassert_equal(err, 0, "Error: %d", err);

	session_done();
	session_init(false);

	/* Test first write */
	ztest_expect_value(uart_dfu_cli_write_send, buf,
		&buf[UART_DFU_HEADER_LENGTH]);
	ztest_expect_value(uart_dfu_cli_write_send, len,
		len - UART_DFU_HEADER_LENGTH);
	ztest_returns_value(uart_dfu_cli_write_send, 0);
	status_post(0);
	err = dfu_target_uart_write(buf, len);
	zassert_equal(err, 0, "Error: %d", err);

	/* Test second write */
	ztest_expect_value(uart_dfu_cli_write_send, buf, buf);
	ztest_expect_value(uart_dfu_cli_write_send, len, len);
	ztest_returns_value(uart_dfu_cli_write_send, 0);
	status_post(0);
	err = dfu_target_uart_write(buf, len);
	zassert_equal(err, 0, "Error: %d", err);

	/* Test API error */
	ztest_expect_value(uart_dfu_cli_write_send, buf, buf);
	ztest_expect_value(uart_dfu_cli_write_send, len, len);
	ztest_returns_value(uart_dfu_cli_write_send, -EINVAL);
	err = dfu_target_uart_write(buf, len);
	zassert_equal(err, -EINVAL, "Error: %d", err);

	/* Test protocol error */
	ztest_expect_value(uart_dfu_cli_write_send, buf, buf);
	ztest_expect_value(uart_dfu_cli_write_send, len, len);
	ztest_returns_value(uart_dfu_cli_write_send, 0);
	evt_post(UART_DFU_CLI_EVT_STOPPED, UART_DFU_CLI_ERR_TIMEOUT);
	err = dfu_target_uart_write(buf, len);
	zassert_equal(err, -ETIMEDOUT, "Error: %d", err);

	session_done();
}

static void test_done(void)
{
	session_init(false);

	/* Test done */
	ztest_expect_value(uart_dfu_cli_done_send, successful, true);
	ztest_returns_value(uart_dfu_cli_done_send, 0);
	ztest_returns_value(uart_dfu_cli_stop, 0);
	status_post(0);
	int err = dfu_target_uart_done(true);
	zassert_equal(err, 0, "Done error: %d", err);

	session_init(false);

	/* Test done while in progress */
	ztest_expect_value(uart_dfu_cli_done_send, successful, true);
	ztest_returns_value(uart_dfu_cli_done_send, 0);
	ztest_returns_value(uart_dfu_cli_stop, -EINPROGRESS);
	evt_post(UART_DFU_CLI_EVT_STOPPED, UART_DFU_CLI_ERR_TIMEOUT);
	err = dfu_target_uart_done(true);
	zassert_equal(err, UART_DFU_CLI_ERR_TIMEOUT, "Done error: %d", err);
}

static void work_init(void)
{
	k_work_init(&status_work, async_status);
	k_work_init(&offset_work, async_offset);
	k_work_q_start(&work_queue,
		       stack_area,
		       K_THREAD_STACK_SIZEOF(stack_area),
		       QUEUE_PRIORITY);
}

void test_main(void)
{
	work_init();

	ztest_test_suite(lib_dfu_target_uart_test,
		ztest_1cpu_unit_test(test_init),
		ztest_1cpu_unit_test(test_offset_get),
		ztest_1cpu_unit_test(test_write),
		ztest_1cpu_unit_test(test_done)
	);

	ztest_run_test_suite(lib_dfu_target_uart_test);
}
