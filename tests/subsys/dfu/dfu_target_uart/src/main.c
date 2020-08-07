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
#include <uart_dfu.h>


#define TEST_FILE_SIZE 		(150 * 1024)

#define UART_DFU_HEADER_LENGTH	4
#define QUEUE_PRIORITY		(-1)
#define QUEUE_STACK_SIZE	512

K_THREAD_STACK_DEFINE(stack_area, QUEUE_STACK_SIZE);
struct k_work_q work_queue;

static struct uart_dfu_cli_cb cli_cb;
static uart_dfu_error_t err_cb;
static void *cli_ctx;

static atomic_t status_ret;
static atomic_t offset_ret;
static atomic_t error_ret;
static struct k_work status_work;
static struct k_work offset_work;


static void async_status(struct k_work *work)
{
	atomic_val_t val;
	ARG_UNUSED(work);
	zassert_not_null(cli_cb.status_cb, "status_cb is NULL");
	zassert_not_null(err_cb, "err_cb is NULL");
	val = atomic_set(&error_ret, 0);
	if (val != 0) {
		err_cb((int) val, cli_ctx);
	} else {
		val = atomic_set(&status_ret, 0);
		cli_cb.status_cb((int) val, cli_ctx);
	}
}

static void async_offset(struct k_work *work)
{
	atomic_val_t val;
	ARG_UNUSED(work);
	zassert_not_null(cli_cb.status_cb, "status_cb is NULL");
	zassert_not_null(cli_cb.offset_cb, "offset_cb is NULL");
	zassert_not_null(err_cb, "err_cb is NULL");
	val = atomic_set(&error_ret, 0);
	if (val != 0) {
		err_cb((int) val, cli_ctx);
		return;
	} 
	val = atomic_set(&status_ret, 0);
	if (val != 0) {
		cli_cb.status_cb((int) val, cli_ctx);
		return;
	}
	val = atomic_set(&offset_ret, 0);
	cli_cb.offset_cb((size_t) val, cli_ctx);
}


int uart_dfu_cli_init(struct uart_dfu_cli *client,
		     struct uart_dfu_cli_cb *callbacks,
		     uart_dfu_error_t error_cb,
		     void *context)
{
	zassert_not_null(client, "Client is NULL");
	zassert_not_null(callbacks, "Callbacks is NULL");
	zassert_not_null(callbacks->offset_cb, "offset_cb is NULL");
	zassert_not_null(callbacks->status_cb, "status_cb is NULL");
	zassert_not_null(error_cb, "error_cb is NULL");

	cli_cb = *callbacks;
	err_cb = error_cb;
	cli_ctx = context;

	return ztest_get_return_value();
}

int uart_dfu_cli_bind(size_t idx, struct uart_dfu_cli *client)
{
	return ztest_get_return_value();
}

int uart_dfu_cli_unbind(size_t idx)
{
	return ztest_get_return_value();
}

int uart_dfu_cli_init_send(size_t idx, size_t file_size)
{
	ztest_check_expected_value(file_size);
	int err = ztest_get_return_value();
	if (err == 0) {
		k_work_submit_to_queue(&work_queue, &status_work);
		return 0;
	}
	return err;
}

int uart_dfu_cli_write_send(size_t idx,
			    const u8_t *const fragment_buf,
			    size_t fragment_size)
{
	ztest_check_expected_value(fragment_buf);
	ztest_check_expected_value(fragment_size);
	int err = ztest_get_return_value();
	if (err == 0) {
		k_work_submit_to_queue(&work_queue, &status_work);
		return 0;
	}
	return err;
}

int uart_dfu_cli_offset_send(size_t idx)
{
	int err = ztest_get_return_value();
	if (err == 0) {
		k_work_submit_to_queue(&work_queue, &offset_work);
		return 0;
	}
	return err;
}

int uart_dfu_cli_done_send(size_t idx, bool successful)
{
	ztest_check_expected_value(successful);
	int err = ztest_get_return_value();
	if (err == 0) {
		k_work_submit_to_queue(&work_queue, &status_work);
		return 0;
	}
	return err;
}

int uart_dfu_cli_stop(size_t idx)
{
	int err = ztest_get_return_value();
	if (err == -EINPROGRESS) {
		k_work_submit_to_queue(&work_queue, &status_work);
	}
	return err;
}

static void dfu_target_callback(enum dfu_target_evt_id evt_id)
{
	ztest_check_expected_value(evt_id);
}

static void state_reset(bool first)
{
	if (first) {
		memset(&cli_cb, 0, sizeof(cli_cb));
		err_cb = NULL;
		cli_ctx = NULL;
	}
	error_ret = 0;
	status_ret = 0;
	offset_ret = 0;
}

static void session_init(bool first)
{
	int err;
	size_t file_size = TEST_FILE_SIZE;
	size_t file_size_exp = file_size - UART_DFU_HEADER_LENGTH;

	state_reset(first);
	if (first) {
		ztest_returns_value(uart_dfu_cli_init, 0);
		ztest_returns_value(uart_dfu_cli_bind, 0);
	}	
	ztest_expect_value(uart_dfu_cli_init_send, file_size, file_size_exp);
	ztest_returns_value(uart_dfu_cli_init_send, 0);
	atomic_set(&status_ret, 0);
	err = dfu_target_uart_init(file_size, dfu_target_callback);
	zassert_equal(err, 0, "Invalid init value: %d", err);
}

static void session_done(void)
{
	int err;

	ztest_expect_value(uart_dfu_cli_done_send, successful, true);
	ztest_returns_value(uart_dfu_cli_done_send, 0);
	ztest_returns_value(uart_dfu_cli_stop, 0);
	atomic_set(&status_ret, 0);
	err = dfu_target_uart_done(true);
	zassert_equal(err, 0, "Done error: %d", err);
} 

static void test_init(void)
{
	int err;
	size_t file_size = TEST_FILE_SIZE;
	size_t file_size_exp = file_size - UART_DFU_HEADER_LENGTH;

	session_init(true);

	/* Test API error */
	ztest_expect_value(uart_dfu_cli_init_send, file_size, file_size_exp);
	ztest_returns_value(uart_dfu_cli_init_send, -EINVAL);
	err = dfu_target_uart_init(file_size, dfu_target_callback);
	zassert_equal(err, -EINVAL, "Invalid init value: %d", err);

	/* Test protocol error */
	ztest_expect_value(uart_dfu_cli_init_send, file_size, file_size_exp);
	ztest_returns_value(uart_dfu_cli_init_send, 0);
	atomic_set(&error_ret, -ETIMEDOUT);
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
	atomic_set(&offset_ret, offset_exp);
	atomic_set(&status_ret, 0);
	err = dfu_target_uart_offset_get(&offset);
	zassert_equal(err, 0, "Error: %d", err);
	zassert_equal(offset, offset_exp, "Unexpected offset");

	/* Test API error */
	ztest_returns_value(uart_dfu_cli_offset_send, -EINVAL);
	err = dfu_target_uart_offset_get(&offset);
	zassert_equal(err, -EINVAL, "Unexpected error %d", err);

	/* Test protocol error */
	ztest_returns_value(uart_dfu_cli_offset_send, 0);
	atomic_set(&error_ret, -ETIMEDOUT);
	err = dfu_target_uart_offset_get(&offset);
	zassert_equal(err, -ETIMEDOUT, "Unexpected error: %d", err);

	session_done();
}

static void test_write(void)
{
	int err;
	u8_t buf[1024];
	size_t len = sizeof(buf);

	session_init(false);

	/* Test short write with no data */
	err = dfu_target_uart_write(buf, UART_DFU_HEADER_LENGTH);
	zassert_equal(err, 0, "Error: %d", err);

	session_done();
	session_init(false);

	/* Test first write */
	ztest_expect_value(uart_dfu_cli_write_send, fragment_buf,
		&buf[UART_DFU_HEADER_LENGTH]);
	ztest_expect_value(uart_dfu_cli_write_send, fragment_size,
		len - UART_DFU_HEADER_LENGTH);
	ztest_returns_value(uart_dfu_cli_write_send, 0);
	atomic_set(&status_ret, 0);
	err = dfu_target_uart_write(buf, len);
	zassert_equal(err, 0, "Error: %d", err);

	/* Test second write */
	ztest_expect_value(uart_dfu_cli_write_send, fragment_buf, buf);
	ztest_expect_value(uart_dfu_cli_write_send, fragment_size, len);
	ztest_returns_value(uart_dfu_cli_write_send, 0);
	atomic_set(&status_ret, 0);
	err = dfu_target_uart_write(buf, len);
	zassert_equal(err, 0, "Error: %d", err);

	/* Test API error */
	ztest_expect_value(uart_dfu_cli_write_send, fragment_buf, buf);
	ztest_expect_value(uart_dfu_cli_write_send, fragment_size, len);
	ztest_returns_value(uart_dfu_cli_write_send, -EINVAL);
	err = dfu_target_uart_write(buf, len);
	zassert_equal(err, -EINVAL, "Error: %d", err);

	/* Test protocol error */
	ztest_expect_value(uart_dfu_cli_write_send, fragment_buf, buf);
	ztest_expect_value(uart_dfu_cli_write_send, fragment_size, len);
	ztest_returns_value(uart_dfu_cli_write_send, 0);
	atomic_set(&status_ret, -ETIMEDOUT);
	err = dfu_target_uart_write(buf, len);
	zassert_equal(err, -ETIMEDOUT, "Error: %d", err);

	session_done();
}

static void test_done(void)
{
	int err;

	session_init(false);

	/* Test done */
	ztest_expect_value(uart_dfu_cli_done_send, successful, true);
	ztest_returns_value(uart_dfu_cli_done_send, 0);
	ztest_returns_value(uart_dfu_cli_stop, 0);
	atomic_set(&status_ret, 0);
	err = dfu_target_uart_done(true);
	zassert_equal(err, 0, "Done error: %d", err);

	session_init(false);
	
	/* Test in progress */
	ztest_expect_value(uart_dfu_cli_done_send, successful, true);
	ztest_returns_value(uart_dfu_cli_done_send, 0);
	ztest_returns_value(uart_dfu_cli_stop, -EINPROGRESS);
	atomic_set(&error_ret, -ECANCELED);
	atomic_set(&status_ret, 0);
	err = dfu_target_uart_done(true);
	zassert_equal(err, 0, "Done error: %d", err);
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