/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-BSD-5-Clause-Nordic
 */
#include <string.h>
#include <zephyr/types.h>
#include <stdbool.h>
#include <ztest.h>
/*
#include <dfu_target.h>
#include <dfu_target_uart.h>
*/
#include <cobs.h>


static void test_cobs_encode_decode_normal(void)
{
	u8_t decoded[] = {
		0x01, 0x02, 0x03
	};
	size_t decoded_size = sizeof(decoded);
	u8_t encoded[COBS_MAX_BYTES];
	size_t encoded_size;
	int err_code;

	/* Test encoding */	
	err_code = cobs_encode(encoded, &encoded_size, decoded, decoded_size);
	zassert_equal(err_code, 0, "Should not return an error.");
	zassert_equal(encoded_size,
				  decoded_size + COBS_OVERHEAD_BYTES,
				  "COBS encoding should add a constant overhead.");
	zassert_equal(encoded[0],
				  encoded_size - 1,
				  "Overhead byte should point to the delimiter if there are no other zeroes in the frame.");
	zassert_equal(encoded[encoded_size - 1],
				  COBS_DELIMITER,
				  "COBS should add a delimiter at the end of the frame");
	zassert_mem_equal(&encoded[1],
					  &decoded[0],
					  decoded_size,
					  "COBS should not modify frame contents if there are no zeroes.");

	/* Test decoding */
	u8_t decoded_copy[COBS_MAX_BYTES];
	memset(decoded_copy, 0xFF, sizeof(decoded_copy));
	decoded_size = 0;
	err_code = cobs_decode(decoded_copy, &decoded_size, encoded, encoded_size);
	zassert_equal(err_code, 0, "Should not return an error. Got: %d.", err_code);
	zassert_equal(decoded_size, sizeof(decoded), "COBS decoding should result in the original length.");
	zassert_mem_equal(decoded_copy,
					  decoded,
					  decoded_size,
					  "COBS decoding should return the original value.");
}

/* TODO: Add more tests. */


#if 0
static void test_dfu_ctx_mcuboot_set_b1_file(void)
{
	int err;
	const char *update;
	bool s0_active = true;

	memcpy(buf, S0_S1, sizeof(S0_S1));
	err = dfu_ctx_mcuboot_set_b1_file(buf, s0_active, &update);

	zassert_equal(err, 0, NULL);
	zassert_true(strcmp("s1", update) == 0, NULL);

	s0_active = false;
	err = dfu_ctx_mcuboot_set_b1_file(buf, s0_active, &update);

	zassert_equal(err, 0, NULL);
	zassert_true(strcmp("s0", update) == 0, NULL);
}

static void test_dfu_ctx_mcuboot_set_b1_file__no_separator(void)
{
	int err;
	const char *update;
	bool s0_active = true;

	memcpy(buf, NO_SPACE, sizeof(NO_SPACE));
	err = dfu_ctx_mcuboot_set_b1_file(buf, s0_active, &update);

	zassert_equal(err, 0, "Should not get error when missing separator");
	zassert_equal(update, NULL, "update should be NULL when no separator");
}

static void test_dfu_ctx_mcuboot_set_b1_file__null(void)
{
	int err;
	const char *update;
	bool s0_active = true;

	err = dfu_ctx_mcuboot_set_b1_file(NULL, s0_active, &update);
	zassert_true(err < 0, NULL);

	err = dfu_ctx_mcuboot_set_b1_file(buf, s0_active, NULL);
	zassert_true(err < 0, NULL);
}

static void test_dfu_ctx_mcuboot_set_b1_file__not_terminated(void)
{
	int err;
	const char *update;
	bool s0_active = true;

	/* Remove any null terminator */
	for (int i = 0; i < sizeof(buf); ++i) {
		buf[i] = 'a';
	}
	err = dfu_ctx_mcuboot_set_b1_file(buf, s0_active, &update);
	zassert_true(err < 0, NULL);
}

static void test_dfu_ctx_mcuboot_set_b1_file__empty(void)
{
	int err;
	const char *update;
	bool s0_active = true;

	err = dfu_ctx_mcuboot_set_b1_file("", s0_active, &update);
	zassert_true(update == NULL, "update should not be set");
}
#endif

void test_main(void)
{
	ztest_test_suite(lib_dfu_target_uart_test,
	     ztest_unit_test(test_cobs_encode_decode_normal)
	 );

	ztest_run_test_suite(lib_dfu_target_uart_test);
}
