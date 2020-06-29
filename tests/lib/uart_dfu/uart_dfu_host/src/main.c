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
#include <uart_dfu_srv.h>
#include <dfu_target.h>


#define TEST_FILE_SIZE 	(150 * 1024)

static struct uart_dfu_srv_cb srv_cb;


void srv_started_send(void)
{
	struct uart_dfu_srv_evt evt = {
		.type = UART_DFU_SRV_EVT_STARTED,
		.err = UART_DFU_SRV_SUCCESS
	};
	srv_cb.evt_cb(&evt);
}

void srv_stopped_send(enum uart_dfu_srv_err err)
{
	struct uart_dfu_srv_evt evt = {
		.type = UART_DFU_SRV_EVT_STOPPED,
		.err = err
	};
	srv_cb.evt_cb(&evt);
}

int uart_dfu_srv_init(uint8_t *buf,
		      size_t max_len,
		      struct uart_dfu_srv_cb *callbacks)
{
	zassert_not_null(buf, NULL);
	zassert_not_null(callbacks, NULL);

	srv_cb = *callbacks;
	return ztest_get_return_value();
}

void uart_dfu_srv_enable(void)
{
}

int uart_dfu_srv_disable(void)
{
	int err = ztest_get_return_value();
	if (err == -EINPROGRESS) {
		srv_stopped_send(UART_DFU_SRV_SUCCESS);
	}
	return err;
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
	ztest_returns_value(uart_dfu_srv_init, 0);
	uart_dfu_host_init();
	uart_dfu_host_enable();
}

static void initialize(size_t file_size, uint8_t *buf, size_t len)
{
	int err;
	err = srv_cb.init_cb(file_size);
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
		err = srv_cb.write_cb(buf, len);
		zassert_equal(err, 0, NULL);
	}
}

static void teardown(void)
{
	ztest_returns_value(uart_dfu_srv_disable, 0);
	uart_dfu_host_disable();
}

static void test_init(void)
{
	int err;
	size_t file_size = TEST_FILE_SIZE;
	setup();
	err = srv_cb.init_cb(file_size);
	zassert_equal(err, 0, NULL);
	err = srv_cb.init_cb(file_size);
	zassert_equal(err, -EBUSY, NULL);
	teardown();
}

static void test_offset(void)
{
	int err;
	uint8_t buf[1024];
	size_t len = sizeof(buf);
	size_t file_size = TEST_FILE_SIZE;
	size_t offset = 1;

	setup();

	/* Test offset without initialization */
	err = srv_cb.offset_cb(&offset);
	zassert_equal(err, 0, NULL);
	zassert_equal(offset, 0, NULL);

	initialize(file_size, buf, len);

	/* Test offset after initialization */
	ztest_returns_value(offset_get, len);
	ztest_returns_value(dfu_target_offset_get, 0);
	err = srv_cb.offset_cb(&offset);
	zassert_equal(err, 0, NULL);
	zassert_equal(offset, len, NULL);

	teardown();
}

static void test_write(void)
{
	int err;
	size_t file_size = TEST_FILE_SIZE;
	uint8_t buf[1024];
	size_t len = sizeof(buf);

	setup();

	/* Test write without init */
	err = srv_cb.write_cb(buf, len);
	zassert_not_equal(err, 0, NULL);

	/* Test write */
	initialize(file_size, buf, len);

	teardown();
}

static void test_done(void)
{
	int err;
	size_t file_size = TEST_FILE_SIZE;
	uint8_t buf[1024];
	size_t len = sizeof(buf);
	size_t offset = 1;

	setup();
	initialize(file_size, buf, len);

	/* Test unsuccessful done (should not clear state). */
	ztest_expect_value(dfu_target_done, successful, false);
	ztest_returns_value(dfu_target_done, 0);
	err = srv_cb.done_cb(false);
	zassert_equal(err, 0, NULL);
	ztest_returns_value(offset_get, len);
	ztest_returns_value(dfu_target_offset_get, 0);
	err = srv_cb.offset_cb(&offset);
	zassert_equal(err, 0, NULL);
	zassert_equal(offset, len, NULL);

	/* Test successful done */
	ztest_expect_value(dfu_target_done, successful, true);
	ztest_returns_value(dfu_target_done, 0);
	err = srv_cb.done_cb(true);
	zassert_equal(err, 0, NULL);
	err = srv_cb.offset_cb(&offset);
	zassert_equal(err, 0, NULL);
	zassert_equal(offset, 0, NULL);

	teardown();
}

static void test_disable(void)
{
	size_t file_size = TEST_FILE_SIZE;
	uint8_t buf[128];
	size_t len = sizeof(buf);

	setup();

	/* Test async abort before init */
	srv_started_send();
	srv_stopped_send(UART_DFU_SRV_ERR_TIMEOUT);

	/* Test async abort after init */
	initialize(file_size, buf, len);
	srv_started_send();
	ztest_returns_value(dfu_target_reset, 0);
	srv_stopped_send(UART_DFU_SRV_ERR_TIMEOUT);

	/* Test user disable */
	initialize(file_size, buf, len);
	srv_started_send();
	ztest_returns_value(dfu_target_reset, 0);
	ztest_returns_value(uart_dfu_srv_disable, -EINPROGRESS);
	uart_dfu_host_disable();
}

void test_main(void)
{
	ztest_test_suite(lib_uart_dfu_target_server_test,
		ztest_unit_test(test_init),
		ztest_unit_test(test_offset),
		ztest_unit_test(test_write),
		ztest_unit_test(test_done),
		ztest_unit_test(test_disable)
	);

	ztest_run_test_suite(lib_uart_dfu_target_server_test);
}
