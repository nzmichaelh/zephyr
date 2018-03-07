/*
 * Copyright (c) 2016, Conor Patrick
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 notice,
 *    this list of conditions and the following disclaimer in the
 documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _U2F_H_
#define _U2F_H_

#include <stdint.h>

#define U2F_EC_FMT_UNCOMPRESSED 0x04

#define U2F_EC_POINT_SIZE 32
#define U2F_EC_PUBKEY_SIZE 65
#define U2F_APDU_SIZE 7
#define U2F_CHALLENGE_SIZE 32
#define U2F_APPLICATION_SIZE 32
#define U2F_KEY_HANDLE_ID_SIZE 8
#define U2F_KEY_HANDLE_KEY_SIZE 36
#define U2F_KEY_HANDLE_SIZE (U2F_KEY_HANDLE_KEY_SIZE + U2F_KEY_HANDLE_ID_SIZE)
#define U2F_REGISTER_REQUEST_SIZE (U2F_CHALLENGE_SIZE + U2F_APPLICATION_SIZE)
#define U2F_MAX_REQUEST_PAYLOAD                                              \
	(1 + U2F_CHALLENGE_SIZE + U2F_APPLICATION_SIZE + 1 +                 \
	 U2F_KEY_HANDLE_SIZE)

/* U2F native commands */
#define U2F_REGISTER 0x01
#define U2F_AUTHENTICATE 0x02
#define U2F_VERSION 0x03
#define U2F_VENDOR_FIRST 0xc0
#define U2F_VENDOR_LAST 0xff

/* U2F_CMD_REGISTER command defines */
#define U2F_REGISTER_ID 0x05
#define U2F_REGISTER_HASH_ID 0x00

/* U2F Authenticate */
#define U2F_AUTHENTICATE_CHECK 0x7
#define U2F_AUTHENTICATE_SIGN 0x3

/* Command status responses */
#define U2F_SW_NO_ERROR 0x9000
#define U2F_SW_WRONG_DATA 0x6984
#define U2F_SW_CONDITIONS_NOT_SATISFIED 0x6985
#define U2F_SW_INS_NOT_SUPPORTED 0x6d00
#define U2F_SW_WRONG_LENGTH 0x6700
#define U2F_SW_CLASS_NOT_SUPPORTED 0x6E00
#define U2F_SW_WRONG_PAYLOAD 0x6a80
#define U2F_SW_INSUFFICIENT_MEMORY 0x9210

/* Delay in milliseconds to wait for user input */
#define U2F_MS_USER_INPUT_WAIT 3000

struct u2f_request_apdu {
	u8_t cla;
	u8_t ins;
	u8_t p1;
	u8_t p2;
	u8_t LC1;
	u8_t LC2;
	u8_t LC3;
	u8_t payload[U2F_MAX_REQUEST_PAYLOAD];
};

struct u2f_ec_point {
	u8_t fmt;
	u8_t x[U2F_EC_POINT_SIZE];
	u8_t y[U2F_EC_POINT_SIZE];
};

struct u2f_register_request {
	u8_t chal[U2F_CHALLENGE_SIZE];
	u8_t app[U2F_APPLICATION_SIZE];
};

struct u2f_authenticate_request {
	u8_t chal[U2F_CHALLENGE_SIZE];
	u8_t app[U2F_APPLICATION_SIZE];
	u8_t khl;
	u8_t kh[U2F_KEY_HANDLE_SIZE];
};

/* u2f_request send a U2F message to U2F protocol */
/*  @req U2F message */
void u2f_request(struct u2f_request_apdu *req);

#endif /* U2F_H_ */
