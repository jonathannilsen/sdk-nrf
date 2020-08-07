/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#include <string.h>
#include <zephyr/types.h>
#include <stdbool.h>
#include <ztest.h>
#include <uart_dfu_host.h>
#include <dfu_target.h>


#define TEST_FILE_SIZE 	(150 * 1024)

static struct uart_dfu_host host;

static struct uart_dfu_srv_cb srv_cb;
static uart_dfu_error_t err_cb;
void *srv_ctx;


int uart_dfu_srv_init(struct uart_dfu_srv *server,
		      u8_t *fragment_buf,
		      size_t fragment_max_size,
		      struct uart_dfu_srv_cb *callbacks,
		      uart_dfu_error_t error_cb,
		      void *context)
{
	zassert_not_null(server, NULL);
	zassert_not_null(fragment_buf, NULL);
	zassert_equal(fragment_max_size,
		      CONFIG_UART_DFU_HOST_MAX_FRAGMENT_SIZE,
		      NULL);
	zassert_not_null(callbacks, NULL);
	zassert_not_null(error_cb, NULL);

	srv_cb = *callbacks;
	err_cb = error_cb;
	srv_ctx = context;

	return ztest_get_return_value();
}

int uart_dfu_srv_bind(size_t idx, struct uart_dfu_srv *server)
{
	return ztest_get_return_value();
}

int uart_dfu_srv_unbind(size_t idx)
{
	return ztest_get_return_value();
}

int uart_dfu_srv_enable(size_t idx)
{
	return ztest_get_return_value();
}

int uart_dfu_srv_disable(size_t idx)
{
	return ztest_get_return_value();
}

int dfu_target_img_type(const void *const buf, size_t len)
{
	ztest_check_expected_value(buf);
	ztest_check_expected_value(len);
	return ztest_get_return_value();
}

int dfu_target_init(int img_type, size_t file_size, dfu_target_callback_t cb)
{
	ztest_check_expected_value(img_type);
	ztest_check_expected_value(file_size);
	return ztest_get_return_value();
}

size_t offset_get(void)
{
	return ztest_get_return_value();
}

int dfu_target_offset_get(size_t *offset)
{
	zassert_not_null(offset, NULL);
	*offset = offset_get();
	return ztest_get_return_value();
}

int dfu_target_write(const void *const buf, size_t len)
{
	ztest_check_expected_value(buf);
	ztest_check_expected_value(len);
	return ztest_get_return_value();
}

int dfu_target_done(bool successful)
{
	ztest_check_expected_value(successful);
	return ztest_get_return_value();
}

int dfu_target_reset(void)
{
	return ztest_get_return_value();
}


static void setup(void)
{
	int err;
	ztest_returns_value(uart_dfu_srv_init, 0);
	ztest_returns_value(uart_dfu_srv_bind, 0);
	err = uart_dfu_host_init(&host, 0);
	zassert_equal(err, 0, NULL);
	ztest_returns_value(uart_dfu_srv_enable, 0);
	err = uart_dfu_host_enable(&host);
	zassert_equal(err, 0, NULL);
}

static void initialize(size_t file_size, u8_t *buf, size_t len)
{
	int err;
	err = srv_cb.init_cb(file_size, srv_ctx);
	zassert_equal(err, 0, NULL);
	if (buf != NULL) {
		ztest_expect_value(dfu_target_img_type, buf, buf);
		ztest_expect_value(dfu_target_img_type, len, len);
		ztest_returns_value(dfu_target_img_type,
				    DFU_TARGET_IMAGE_TYPE_MCUBOOT);
		ztest_expect_value(dfu_target_init,
				   img_type,
				   DFU_TARGET_IMAGE_TYPE_MCUBOOT);
		ztest_expect_value(dfu_target_init,
				   file_size,
				   file_size);
		ztest_returns_value(dfu_target_init, 0);
		ztest_expect_value(dfu_target_write, buf, buf);
		ztest_expect_value(dfu_target_write, len, len);
		ztest_returns_value(dfu_target_write, 0);
		err = srv_cb.write_cb(buf, len, srv_ctx);
		zassert_equal(err, 0, NULL);	
	}
}


static void teardown(void)
{
	int err;
	ztest_returns_value(uart_dfu_srv_disable, 0);
	err = uart_dfu_host_disable(&host);
	zassert_equal(err, 0, NULL);
}

static void test_init(void)
{
	int err;
	size_t file_size = TEST_FILE_SIZE;
	setup();
	err = srv_cb.init_cb(file_size, srv_ctx);
	zassert_equal(err, 0, NULL);
	err = srv_cb.init_cb(file_size, srv_ctx);
	zassert_equal(err, -EBUSY, NULL);
	teardown();
}

static void test_offset(void)
{
	int err;
	u8_t buf[1024];
	size_t len = sizeof(buf);
	size_t file_size = TEST_FILE_SIZE; 
	size_t offset = 1;

	setup();
	
	/* Test offset without initialization */
	err = srv_cb.offset_cb(&offset, srv_ctx);
	zassert_equal(err, 0, NULL);
	zassert_equal(offset, 0, NULL);

	initialize(file_size, buf, len);	

	/* Test offset after initialization */
	ztest_returns_value(offset_get, len);
	ztest_returns_value(dfu_target_offset_get, 0);
	err = srv_cb.offset_cb(&offset, srv_ctx);
	zassert_equal(err, 0, NULL);
	zassert_equal(offset, len, NULL);

	teardown();
}

static void test_write(void)
{
	int err;
	size_t file_size = TEST_FILE_SIZE; 
	u8_t buf[1024];
	size_t len = sizeof(buf);

	setup();

	/* Test write without init */
	err = srv_cb.write_cb(buf, len, srv_ctx);
	zassert_not_equal(err, 0, NULL);

	/* Test write */
	initialize(file_size, buf, len);

	teardown();
}

static void test_done(void)
{
	int err;
	size_t file_size = TEST_FILE_SIZE; 
	u8_t buf[1024];
	size_t len = sizeof(buf);
	size_t offset = 1;

	setup();
	initialize(file_size, buf, len);

	/* Test unsuccessful done (should not clear state). */
	ztest_expect_value(dfu_target_done, successful, false);
	ztest_returns_value(dfu_target_done, 0);
	err = srv_cb.done_cb(false, srv_ctx);
	zassert_equal(err, 0, NULL);
	ztest_returns_value(offset_get, len);
	ztest_returns_value(dfu_target_offset_get, 0);
	err = srv_cb.offset_cb(&offset, srv_ctx);
	zassert_equal(err, 0, NULL);
	zassert_equal(offset, len, NULL);

	/* Test successful done */
	ztest_expect_value(dfu_target_done, successful, true);
	ztest_returns_value(dfu_target_done, 0);
	err = srv_cb.done_cb(true, srv_ctx);
	zassert_equal(err, 0, NULL);
	err = srv_cb.offset_cb(&offset, srv_ctx);
	zassert_equal(err, 0, NULL);
	zassert_equal(offset, 0, NULL);

	teardown();
}

static void test_error(void)
{
	setup();
	/* TODO: add test when session timeout is added */
	teardown();
}

void test_main(void)
{
	ztest_test_suite(lib_uart_dfu_target_server_test,
		ztest_unit_test(test_init),
		ztest_unit_test(test_offset),
		ztest_unit_test(test_write),
		ztest_unit_test(test_done),
		ztest_unit_test(test_error)
	);

	ztest_run_test_suite(lib_uart_dfu_target_server_test);
}
