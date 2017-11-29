/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>

#include <drivers/crc/crc16_sw.c>
#include <drivers/crc/crc7_sw.c>

void test_crc16(void)
{
	u8_t test0[] = { };
	u8_t test1[] = { 'A' };
	u8_t test2[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };

	zassert(crc16_ccitt(test0, sizeof(test0)) == 0x1d0f, "pass", "fail");
	zassert(crc16_ccitt(test1, sizeof(test1)) == 0x9479, "pass", "fail");
	zassert(crc16_ccitt(test2, sizeof(test2)) == 0xe5cc, "pass", "fail");
}

void test_crc7(void)
{
	u8_t test0[] = { };
	u8_t test1[] = { 'A' };
	u8_t test2[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
	u8_t test3[] = { 0x77, 0, 0, 0, 0 };

	zassert_equal(crc7(0, test0, sizeof(test0)), 0, NULL);
	zassert_equal(crc7(0, test1, sizeof(test1)), 0x6d, NULL);
	zassert_equal(crc7(0, test2, sizeof(test2)), 0x75, NULL);
	zassert_equal(crc7(0, test3, sizeof(test3)), 0x32, NULL);
}

void test_main(void)
{
	ztest_test_suite(test_crc16,
			 ztest_unit_test(test_crc16),
			 ztest_unit_test(test_crc7)
			 );
	ztest_run_test_suite(test_crc16);
}
