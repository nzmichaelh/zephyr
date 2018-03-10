/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL 4
#define SYS_LOG_DOMAIN "u2f"
#include <logging/sys_log.h>

#include <misc/byteorder.h>
#include <net/buf.h>
#include <string.h>
#include <zephyr.h>

#include <tinycrypt/constants.h>
#include <tinycrypt/ecc_dh.h>
#include <tinycrypt/ecc_dsa.h>
#include <tinycrypt/sha256.h>

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

static const char attestation_key[] =
	"\xf3\xfc\xcc\x0d\x00\xd8\x03\x19\x54\xf9"
	"\x08\x64\xd4\x3c\x24\x7f\x4b\xf5\xf0\x66\x5c\x6b\x50\xcc"
	"\x17\x74\x9a\x27\xd1\xcf\x76\x64";

static const char attestation_der[] =
	"\x30\x82\x01\x3c\x30\x81\xe4\xa0\x03\x02"
	"\x01\x02\x02\x0a\x47\x90\x12\x80\x00\x11\x55\x95\x73\x52"
	"\x30\x0a\x06\x08\x2a\x86\x48\xce\x3d\x04\x03\x02\x30\x17"
	"\x31\x15\x30\x13\x06\x03\x55\x04\x03\x13\x0c\x47\x6e\x75"
	"\x62\x62\x79\x20\x50\x69\x6c\x6f\x74\x30\x1e\x17\x0d\x31"
	"\x32\x30\x38\x31\x34\x31\x38\x32\x39\x33\x32\x5a\x17\x0d"
	"\x31\x33\x30\x38\x31\x34\x31\x38\x32\x39\x33\x32\x5a\x30"
	"\x31\x31\x2f\x30\x2d\x06\x03\x55\x04\x03\x13\x26\x50\x69"
	"\x6c\x6f\x74\x47\x6e\x75\x62\x62\x79\x2d\x30\x2e\x34\x2e"
	"\x31\x2d\x34\x37\x39\x30\x31\x32\x38\x30\x30\x30\x31\x31"
	"\x35\x35\x39\x35\x37\x33\x35\x32\x30\x59\x30\x13\x06\x07"
	"\x2a\x86\x48\xce\x3d\x02\x01\x06\x08\x2a\x86\x48\xce\x3d"
	"\x03\x01\x07\x03\x42\x00\x04\x8d\x61\x7e\x65\xc9\x50\x8e"
	"\x64\xbc\xc5\x67\x3a\xc8\x2a\x67\x99\xda\x3c\x14\x46\x68"
	"\x2c\x25\x8c\x46\x3f\xff\xdf\x58\xdf\xd2\xfa\x3e\x6c\x37"
	"\x8b\x53\xd7\x95\xc4\xa4\xdf\xfb\x41\x99\xed\xd7\x86\x2f"
	"\x23\xab\xaf\x02\x03\xb4\xb8\x91\x1b\xa0\x56\x99\x94\xe1"
	"\x01\x30\x0a\x06\x08\x2a\x86\x48\xce\x3d\x04\x03\x02\x03"
	"\x47\x00\x30\x44\x02\x20\x60\xcd\xb6\x06\x1e\x9c\x22\x26"
	"\x2d\x1a\xac\x1d\x96\xd8\xc7\x08\x29\xb2\x36\x65\x31\xdd"
	"\xa2\x68\x83\x2c\xb8\x36\xbc\xd3\x0d\xfa\x02\x20\x63\x1b"
	"\x14\x59\xf0\x9e\x63\x30\x05\x57\x22\xc8\xd8\x9b\x7f\x48"
	"\x88\x3b\x90\x89\xb8\x8d\x60\xd1\xd9\x79\x59\x02\xb3\x04"
	"\x10\xdf";

struct slice {
	const u8_t *p;
	int len;
};

const u8_t *get_p(struct slice *s, int offset, int len)
{
	if (offset < 0 || len < 0) {
		return NULL;
	}
	if (offset + len > s->len) {
		return NULL;
	}
	return s->p + offset;
}

int get_u8(struct slice *s, int offset)
{
	const u8_t *p = get_p(s, offset, 1);

	if (p == NULL) {
		return -EINVAL;
	}
	return *p;
}

void dump_hex(const char *msg, const u8_t *buf, int len)
{
	printk("%s(%d): ", msg, len);
	for (int i = 0; i < len; i++) {
		printk(" %x", buf[i]);
	}
	printk("\n");
}

static int net_buf_add_varint(struct net_buf *resp, const u8_t *buf, int len)
{
	net_buf_add_u8(resp, 0x02);

	if ((buf[0] & 0x80) != 0) {
		net_buf_add_le16(resp, len + 1);
		net_buf_add_mem(resp, buf, len);

		return len + 1 + 2;
	}

	net_buf_add_u8(resp, len);
	net_buf_add_mem(resp, buf, len);

	return len + 1 + 1;
}

static void net_buf_add_x962(struct net_buf *resp, const u8_t *signature)
{
	/* Encode the signature in X9.62 format */
	net_buf_add_u8(resp, 0x30);
	u8_t *len = net_buf_add(resp, 1);

	*len = 0;

	*len += net_buf_add_varint(resp, &signature[0], 32);
	*len += net_buf_add_varint(resp, &signature[32], 32);
}

static int u2f_authenticate(int p1, struct slice *pc, int le,
			       struct net_buf *resp)
{
	const u8_t *chal = get_p(pc, 0, 32);
	const u8_t *app = get_p(pc, 32, 32);
	int l = get_u8(pc, 64);
	const u8_t *handle = get_p(pc, 65, l);
	u8_t signature[64];

	SYS_LOG_DBG("chal=%p app=%p l=%d handle=%p", chal, app, l, handle);

	if (chal == NULL || app == NULL || l < 0 || handle == NULL) {
		return -EINVAL;
	}

	dump_hex("chal", chal, 32);
	dump_hex("app", app, 32);
	dump_hex("handle", handle, l);

	/* Add user presence */
	net_buf_add_u8(resp, 1);

	/* Add the press counter */
	net_buf_add_be32(resp, 1234);

	/* Add the signature */
	net_buf_add_x962(resp, signature);

	return U2F_SW_NO_ERROR;
}

static int u2f_register(int p1, struct slice *pc, int le,
			   struct net_buf *resp)
{
	const u8_t *chal = get_p(pc, 0, 32);
	const u8_t *app = get_p(pc, 32, 32);
	u8_t private[32];
	u8_t ch;

	if (pc->len != 64) {
		SYS_LOG_ERR("lc=%d", pc->len);
		return U2F_SW_WRONG_LENGTH;
	}

	SYS_LOG_DBG("line=%d", __LINE__);

	/* Add the header */
	net_buf_add_u8(resp, 0x05);

	/* Reserve space for the public key */
	net_buf_add_u8(resp, U2F_EC_FMT_UNCOMPRESSED);
	u8_t *public = net_buf_add(resp, 64);

	SYS_LOG_DBG("line=%d", __LINE__);

	/* Generate a new public/private key pair */
	if (uECC_make_key(public, private, uECC_secp256r1()) !=
	    TC_CRYPTO_SUCCESS) {
		SYS_LOG_ERR("uECC_make_key");
		return U2F_SW_INS_NOT_SUPPORTED;
	}

	SYS_LOG_DBG("line=%d", __LINE__);

	/* Add the key handle */
	u8_t *handle = private;

	net_buf_add_u8(resp, sizeof(private));
	net_buf_add_mem(resp, handle, sizeof(private));

	/* Add the attestation certificate */
	net_buf_add_mem(resp, attestation_der, sizeof(attestation_der) - 1);

	/* Generate the digest */
	struct tc_sha256_state_struct sha;

	if (tc_sha256_init(&sha) != TC_CRYPTO_SUCCESS) {
		SYS_LOG_ERR("tc_sha256_init");
		return U2F_SW_INS_NOT_SUPPORTED;
	}

	SYS_LOG_DBG("line=%d", __LINE__);

	ch = 0;
	tc_sha256_update(&sha, &ch, sizeof(ch));
	tc_sha256_update(&sha, app, 32);
	tc_sha256_update(&sha, chal, 32);
	tc_sha256_update(&sha, handle, sizeof(private));
	ch = U2F_EC_FMT_UNCOMPRESSED;
	tc_sha256_update(&sha, &ch, sizeof(ch));
	tc_sha256_update(&sha, public, 64);

	u8_t digest[TC_SHA256_DIGEST_SIZE];

	tc_sha256_final(digest, &sha);

	SYS_LOG_DBG("line=%d", __LINE__);

	/* Generate the signature */
	u8_t signature[64];

	if (uECC_sign(attestation_key, digest, sizeof(digest), signature,
		      uECC_secp256r1()) != TC_CRYPTO_SUCCESS) {
		SYS_LOG_ERR("uECC_sign");
		return U2F_SW_INS_NOT_SUPPORTED;
	}

	SYS_LOG_DBG("line=%d", __LINE__);
	net_buf_add_x962(resp, signature);

	return U2F_SW_NO_ERROR;
}

int u2f_dispatch(struct net_buf *req, struct net_buf *resp)
{
	u8_t cla;
	u8_t ins;
	u8_t p1;
	u8_t p2;
	u16_t lc = 0;
	struct slice pc;
	u16_t le = 0;
	int err;

	SYS_LOG_DBG("");

	dump_hex("<<", req->data, req->len);

	cla = net_buf_pull_u8(req);
	if (cla != 0) {
		SYS_LOG_ERR("bad cla");
		return -EINVAL;
	}

	ins = net_buf_pull_u8(req);
	p1 = net_buf_pull_u8(req);
	p2 = net_buf_pull_u8(req);

	if (net_buf_pull_u8(req) != 0) {
		SYS_LOG_ERR("Bad lc header");
		return -EINVAL;
	}

	pc.len = net_buf_pull_be16(req);
	pc.p = req->data;
	req->data += pc.len;
	req->len -= pc.len;

	if (req->len > 0) {
		le = net_buf_pull_be16(req);
	}

	SYS_LOG_DBG("ins=%d p1=%d p2=%d lc=%d le=%d", ins, p1, p2, lc, le);
	switch (ins) {
	case U2F_REGISTER:
		err = u2f_register(p1, &pc, le, resp);
		break;
	case U2F_AUTHENTICATE:
		err = u2f_authenticate(p1, &pc, le, resp);
		break;
	default:
		err = U2F_SW_INS_NOT_SUPPORTED;
		break;
	}

	SYS_LOG_DBG("err=%x", err);

	net_buf_add_be16(resp, err);
	return 0;
}
