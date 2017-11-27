/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>

#include <drivers/crc/crc16_sw.c>
#include <drivers/crc/crc_ccitt_sw.c>

void test_crc16_ccitt(void)
{
	u8_t test0[] = { };
	u8_t test1[] = { 'A' };
	u8_t test2[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };

	zassert(crc16_ccitt(test0, sizeof(test0)) == 0x1d0f, "pass", "fail");
	zassert(crc16_ccitt(test1, sizeof(test1)) == 0x9479, "pass", "fail");
	zassert(crc16_ccitt(test2, sizeof(test2)) == 0xe5cc, "pass", "fail");
}

void test_crc16_ansi(void)
{
	u8_t test0[] = { };
	u8_t test1[] = { 'A' };
	u8_t test2[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };

	zassert(crc16_ansi(test0, sizeof(test0)) == 0x800d, "pass", "fail");
	zassert(crc16_ansi(test1, sizeof(test1)) == 0x8f85, "pass", "fail");
	zassert(crc16_ansi(test2, sizeof(test2)) == 0x9ecf, "pass", "fail");
}

void test_crc_ccitt(void)
{
	u8_t test0[] = { };
	u8_t test1[] = { 'A' };
	u8_t test2[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
	u8_t test3[] = { 'Z', 'e', 'p', 'h', 'y', 'r', 0, 0 };
	u16_t crc;

	zassert_equal(crc_ccitt(0, test0, sizeof(test0)), 0x0, NULL);
	zassert_equal(crc_ccitt(0, test1, sizeof(test1)), 0x538d, NULL);
	zassert_equal(crc_ccitt(0, test2, sizeof(test2)), 0x2189, NULL);

	/* Appending the CRC to a buffer and computing the CRC over
	 * the extended buffer leaves a residual of zero.
	 */
	crc = crc_ccitt(0, test3, sizeof(test3) - sizeof(u16_t));
	test3[sizeof(test3)-2] = (u8_t)(crc >> 0);
	test3[sizeof(test3)-1] = (u8_t)(crc >> 8);

	zassert_equal(crc_ccitt(0, test3, sizeof(test3)), 0, NULL);
}

void test_crc_ccitt_for_ppp(void)
{
	/* Example capture including FCS from
	 * https://www.horo.ch/techno/ppp-fcs/examples_en.html
	 */
	u8_t test0[] = {
		0xff, 0x03, 0xc0, 0x21, 0x01, 0x01, 0x00, 0x17,
		0x02, 0x06, 0x00, 0x0a, 0x00, 0x00, 0x05, 0x06,
		0x00, 0x2a, 0x2b, 0x78, 0x07, 0x02, 0x08, 0x02,
		0x0d, 0x03, 0x06, 0xa5, 0xf8
	};
	u16_t fcs = crc_ccitt(0xffff, test0, sizeof(test0));

	zassert(fcs == 0xf0b8, "pass", "fail");
}

void test_main(void)
{
	ztest_test_suite(test_crc16,
			 ztest_unit_test(test_crc16_ccitt),
			 ztest_unit_test(test_crc16_ansi),
			 ztest_unit_test(test_crc_ccitt),
			 ztest_unit_test(test_crc_ccitt_for_ppp)
		);
	ztest_run_test_suite(test_crc16);
}
