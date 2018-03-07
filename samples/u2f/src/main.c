/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_LEVEL 4
#define SYS_LOG_DOMAIN "main"
#include <logging/sys_log.h>

#include <misc/byteorder.h>
#include <net/buf.h>
#include <string.h>
#include <zephyr.h>

#include <tinycrypt/constants.h>
#include <tinycrypt/ecc_dh.h>
#include <tinycrypt/ecc_dsa.h>
#include <tinycrypt/sha256.h>
#include <usb/class/usb_hid.h>
#include <usb/usb_device.h>

#include "u2f.h"

#define HID_PACKET_SIZE 64

#define U2F_POINT_UNCOMPRESSED 0x04 /* Uncompressed point format */

#define TYPE_MASK 0x80 /* Frame type mask */
#define TYPE_INIT 0x80 /* Initial frame identifier */
#define TYPE_CONT 0x00 /* Continuation frame identifier */

#define U2FHID_PING                                                          \
	(TYPE_INIT | 0x01) /* Echo data through local processor only */
#define U2FHID_MSG (TYPE_INIT | 0x03)   /* Send U2F message frame */
#define U2FHID_LOCK (TYPE_INIT | 0x04)  /* Send lock channel command */
#define U2FHID_INIT (TYPE_INIT | 0x06)  /* Channel initialization */
#define U2FHID_WINK (TYPE_INIT | 0x08)  /* Send device identification wink */
#define U2FHID_ERROR (TYPE_INIT | 0x3f) /* Error response */

#define CAPABILITY_WINK 0x01
#define CAPABILITY_LOCK 0x02

#define U2FHID_BROADCAST 0xffffffff

#define U2FHID_INIT_PAYLOAD_SIZE (HID_PACKET_SIZE - 7)
#define U2FHID_CONT_PAYLOAD_SIZE (HID_PACKET_SIZE - 5)
#define U2FHID_MAX_PAYLOAD_SIZE (7609)

struct u2f_pkt {
	u32_t cid;
	u8_t cmd;
};

struct u2f_init_pkt {
	u32_t cid;
	u8_t cmd;
	u8_t bcnt[2];
	u8_t payload[U2FHID_INIT_PAYLOAD_SIZE];
};

struct u2f_cont_pkt {
	u32_t cid;
	u8_t seq;
	u8_t payload[U2FHID_CONT_PAYLOAD_SIZE];
} __packet;

struct u2f_hdr {
	u32_t cid;
	u8_t cmd;
	u8_t payload[];
};

NET_BUF_POOL_DEFINE(u2f_msg_pool, 4, 700, 0, NULL);

/* Some HID sample Report Descriptor */
static const u8_t hid_report_desc[] = {
	0x06, 0xd0,
	0xf1,       /* USAGE_PAGE (FIDO Alliance) */
	0x09, 0x01, /* USAGE (Keyboard) */
	0xa1, 0x01, /* COLLECTION (Application) */

	0x09, 0x20, /* USAGE (Input Report Data) */
	0x15, 0x00, /* LOGICAL_MINIMUM (0) */
	0x26, 0xff,
	0x00,		       /* LOGICAL_MAXIMUM (255) */
	0x75, 0x08,	    /* REPORT_SIZE (8) */
	0x95, HID_PACKET_SIZE, /* REPORT_COUNT (64) */
	0x81, 0x02,	    /* INPUT (Data,Var,Abs) */
	0x09, 0x21,	    /* USAGE(Output Report Data) */
	0x15, 0x00,	    /* LOGICAL_MINIMUM (0) */
	0x26, 0xff,
	0x00,		       /* LOGICAL_MAXIMUM (255) */
	0x75, 0x08,	    /* REPORT_SIZE (8) */
	0x95, HID_PACKET_SIZE, /* REPORT_COUNT (64) */
	0x91, 0x02,	    /* OUTPUT (Data,Var,Abs) */

	0xc0, /* END_COLLECTION */
};

struct u2f_msg {
	u8_t cla;
	u8_t ins;
	u8_t p1;
	u8_t p2;
	u8_t payload[0];
};

struct u2f_hid_msg {
	u32_t cid;
	u8_t cmd;
	u16_t bcnt;
	u8_t payload[0];
};

struct u2f_data {
	u32_t next_channel;
	u32_t cid;

	struct net_buf *rx;
	u32_t rx_cid;
	u8_t rx_cmd;
	u16_t rx_bcnt;
	u8_t rx_seq;

	struct net_buf *tx;
	int tx_seq;
	struct k_work tx_work;

	struct tc_sha256_state_struct sha256;
};

static struct u2f_data data;

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

int default_CSPRNG(u8_t *dest, unsigned int size)
{
	/* This is not a CSPRNG, but it's the only thing available in the
	 * system at this point in time.
	 */

	while (size) {
		u32_t len = size >= sizeof(u32_t) ? sizeof(u32_t) : size;
		u32_t rv = sys_rand32_get();

		memcpy(dest, &rv, len);
		dest += len;
		size -= len;
	}

	return 1;
}

static void dump_hex(const char *msg, const u8_t *buf, int len)
{
	printk("%s(%d): ", msg, len);
	for (int i = 0; i < len; i++) {
		printk(" %x", buf[i]);
	}
	printk("\n");
}

static int debug_cb(struct usb_setup_packet *setup, s32_t *len, u8_t **data)
{
	SYS_LOG_DBG("Debug callback");

	return -ENOTSUP;
}

static void tx(struct k_work *work)
{
	int err;
	u32_t wrote;
	int at;
	int stride;

	if (data.tx == NULL) {
		return;
	}

	SYS_LOG_DBG("tx_seq=%d len=%d", data.tx_seq, data.tx->len);

	if (data.tx_seq < 0) {
		at = 0;
		stride = HID_PACKET_SIZE;
		err = usb_write(CONFIG_HID_INT_EP_ADDR, data.tx->data,
				HID_PACKET_SIZE, &wrote);
	} else {
		struct u2f_cont_pkt pkt = {
			.cid = *(u32_t *)(data.tx->data), .seq = data.tx_seq,
		};

		at = HID_PACKET_SIZE +
		     (data.tx_seq * U2FHID_CONT_PAYLOAD_SIZE);
		stride = U2FHID_CONT_PAYLOAD_SIZE;
		memcpy(pkt.payload, data.tx->data + at, stride);
		err = usb_write(CONFIG_HID_INT_EP_ADDR, (u8_t *)&pkt,
				sizeof(pkt), &wrote);
	}

	if (err != 0) {
		k_work_submit(&data.tx_work);
	} else {
		data.tx_seq++;

		if (at + stride >= data.tx->len) {
			net_buf_unref(data.tx);
			data.tx = NULL;
		} else {
			k_work_submit(&data.tx_work);
		}
	}
}

static int respond(struct net_buf *resp)
{
	if (data.tx != NULL) {
		SYS_LOG_ERR("tx buffer is full");
		return -EINVAL;
	}

	dump_hex(">>", resp->data, resp->len);
	data.tx = resp;
	data.tx_seq = -1;

	k_work_submit(&data.tx_work);
	return 0;
}

#define U2F_REGISTER 0x01
#define U2F_AUTHENTICATE 0x02
#define U2F_VERSION 0x03

/* The command completed successfully without error. */
#define SW_NO_ERROR 0x9000

/* The request was rejected due to test-of-user-presence being required. */
#define SW_CONDITIONS_NOT_SATISFIED 0x6985

/* The request was rejected due to an invalid key handle. */
#define SW_WRONG_DATA 0x6A80

/* The length of the request was invalid. */
#define SW_WRONG_LENGTH 0x6700

/* The Class byte of the request is not supported. */
#define SW_CLA_NOT_SUPPORTED 0x6E00

/* The Instruction of the request is not supported. */
#define SW_INS_NOT_SUPPORTED 0x6D00

void u2f_sha256_start(void) { tc_sha256_init(&data.sha256); }

void u2f_sha256_update(u8_t *buf, u8_t len)
{
	tc_sha256_update(&data.sha256, buf, len);
}

void u2f_sha256_finish(void) {}

int8_t u2f_get_user_feedback(void) { return 0; }

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

static int handle_authenticate(int p1, u8_t *pc, int lc, int le,
			       struct net_buf *resp)
{
	u8_t signature[64];

	/* Add user presence */
	net_buf_add_u8(resp, 1);

	/* Add the press counter */
	net_buf_add_be32(resp, 1234);

	/* Add the signature */
	net_buf_add_x962(resp, signature);

	return SW_NO_ERROR;
}

static int handle_register(int p1, u8_t *pc, int lc, int le,
			   struct net_buf *resp)
{
	u8_t *chal = pc + 0;
	u8_t *app = pc + 32;
	u8_t private[32];
	u8_t ch;

	if (lc != 64) {
		SYS_LOG_ERR("lc=%d", lc);
		return SW_WRONG_LENGTH;
	}

	/* Add the header */
	net_buf_add_u8(resp, 0x05);

	/* Reserve space for the public key */
	net_buf_add_u8(resp, U2F_POINT_UNCOMPRESSED);
	u8_t *public = net_buf_add(resp, 64);

	/* Generate a new public/private key pair */
	if (uECC_make_key(public, private, uECC_secp256r1()) !=
	    TC_CRYPTO_SUCCESS) {
		SYS_LOG_ERR("uECC_make_key");
		return SW_INS_NOT_SUPPORTED;
	}

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
		return SW_INS_NOT_SUPPORTED;
	}

	ch = 0;
	tc_sha256_update(&sha, &ch, sizeof(ch));
	tc_sha256_update(&sha, app, 32);
	tc_sha256_update(&sha, chal, 32);
	tc_sha256_update(&sha, handle, sizeof(private));
	ch = U2F_POINT_UNCOMPRESSED;
	tc_sha256_update(&sha, &ch, sizeof(ch));
	tc_sha256_update(&sha, public, 64);

	u8_t digest[TC_SHA256_DIGEST_SIZE];

	tc_sha256_final(digest, &sha);

	/* Generate the signature */
	u8_t signature[64];

	if (uECC_sign(attestation_key, digest, sizeof(digest), signature,
		      uECC_secp256r1()) != TC_CRYPTO_SUCCESS) {
		SYS_LOG_ERR("uECC_sign");
		return SW_INS_NOT_SUPPORTED;
	}

	net_buf_add_x962(resp, signature);

	return U2F_SW_NO_ERROR;
}

static int handle_init(struct net_buf *req, struct net_buf *resp)
{
	SYS_LOG_DBG("");

	net_buf_add_mem(resp, net_buf_pull(req, 8), 8);
	net_buf_add_be32(resp, ++data.next_channel);
	net_buf_add_u8(resp, 2);
	net_buf_add_u8(resp, 0);
	net_buf_add_u8(resp, 1);
	net_buf_add_u8(resp, 0);
	net_buf_add_u8(resp, CAPABILITY_WINK);

	return 0;
}

static int handle_msg(struct net_buf *req, struct net_buf *resp)
{
	struct u2f_hdr *hdr;
	u8_t cla;
	u8_t ins;
	u8_t p1;
	u8_t p2;
	u16_t lc = 0;
	u8_t *pc;
	u16_t le = 0;
	int err;

	SYS_LOG_DBG("");

	dump_hex("<<", req->data, req->len);

	hdr = req->data;
	net_buf_pull(req, sizeof(struct u2f_hdr));
	SYS_LOG_DBG("cid=%x cmd=%x", hdr->cid, hdr->cmd);

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

	lc = net_buf_pull_be16(req);
	pc = req->data;
	req->data += lc;
	req->len -= lc;

	if (req->len > 0) {
		le = net_buf_pull_be16(req);
	}

	SYS_LOG_DBG("ins=%d p1=%d p2=%d lc=%d le=%d", ins, p1, p2, lc, le);
	switch (ins) {
	case U2F_REGISTER:
		err = handle_register(p1, pc, lc, le, resp);
		break;
	case U2F_AUTHENTICATE:
		err = handle_authenticate(p1, pc, lc, le, resp);
		break;
	default:
		err = SW_INS_NOT_SUPPORTED;
		break;
	}

	SYS_LOG_DBG("err=%x", err);

	net_buf_add_be16(resp, err);
	return 0;
}

static int handle_req(struct net_buf *req)
{
	struct u2f_hdr *hdr = (struct u2f_hdr *)req->data;
	struct net_buf *resp;
	u8_t *bcnt;
	int err;

	printk("<<");
	for (int i = 0; i < req->len; i++) {
		printk(" %x", req->data[i]);
	}
	printk("\n");

	SYS_LOG_DBG("cid=%x cmd=%x", hdr->cid, hdr->cmd);
	resp = net_buf_alloc(&u2f_msg_pool, K_NO_WAIT);
	if (resp == NULL) {
		SYS_LOG_ERR("no buf");
		return -ENOMEM;
	}

	net_buf_add_mem(resp, &hdr->cid, sizeof(hdr->cid));
	net_buf_add_u8(resp, hdr->cmd);
	bcnt = net_buf_add(resp, 2);

	switch (hdr->cmd) {
	case U2FHID_INIT:
		err = handle_init(req, resp);
		break;
	case U2FHID_MSG:
		err = handle_msg(req, resp);
		break;
	default:
		err = -EINVAL;
	}

	net_buf_unref(req);

	if (err != 0) {
		net_buf_unref(resp);
		return err;
	}

	sys_put_be16(resp->len - 4 - 1 - 2, bcnt);
	return respond(resp);
}

static int set_report_cb(struct usb_setup_packet *setup, s32_t *plen,
			 u8_t **pdata)
{
	struct u2f_pkt *hdr = (struct u2f_pkt *)*pdata;
	int len = *plen;
	int end = 0;

	if (len < 5) {
		return -EINVAL;
	}
	if ((hdr->cmd & TYPE_INIT) != 0) {
		struct u2f_init_pkt *pkt = (struct u2f_init_pkt *)*pdata;
		struct u2f_hdr hdr;
		u16_t bcnt;

		if (len < 7) {
			SYS_LOG_ERR("init packet too short");
			return -EINVAL;
		}
		bcnt = sys_get_be16(pkt->bcnt);
		if (bcnt > U2FHID_MAX_PAYLOAD_SIZE) {
			SYS_LOG_ERR("bcnt too big");
			return -EINVAL;
		}

		SYS_LOG_DBG("cmd=%x bcnt=%d", pkt->cmd, bcnt);
		if (data.rx != NULL) {
			net_buf_unref(data.rx);
			data.rx = NULL;
		}

		data.rx = net_buf_alloc(&u2f_msg_pool, K_NO_WAIT);
		if (data.rx == NULL) {
			SYS_LOG_ERR("No memory");
			return -ENOMEM;
		}

		data.rx_cid = pkt->cid;
		data.rx_bcnt = bcnt;
		data.rx_seq = 0;

		end = len - 7;
		if (end >= data.rx->size) {
			return -EINVAL;
		}
		hdr.cid = pkt->cid;
		hdr.cmd = pkt->cmd;

		net_buf_add_mem(data.rx, &hdr, sizeof(hdr));
		net_buf_add_mem(data.rx, pkt->payload, len - 7);

	} else {
		struct u2f_cont_pkt *pkt = (struct u2f_cont_pkt *)*pdata;
		int offset;

		if (data.rx == NULL) {
			SYS_LOG_ERR("no rx buf");
			return -EINVAL;
		}
		if (pkt->cid != data.rx_cid) {
			SYS_LOG_ERR("cid changed");
			return -EINVAL;
		}
		if (pkt->seq != data.rx_seq) {
			SYS_LOG_ERR("seq out of order");
			return -EINVAL;
		}
		data.rx_seq = pkt->seq + 1;

		offset = U2FHID_INIT_PAYLOAD_SIZE +
			 pkt->seq * U2FHID_CONT_PAYLOAD_SIZE;
		end = offset + len - 5;
		SYS_LOG_DBG("seq=%x offset=%d", pkt->seq, offset);
		if (end >= data.rx->size) {
			return -EINVAL;
		}
		net_buf_add_mem(data.rx, pkt->payload, len - 5);
	}

	if (end >= data.rx_bcnt) {
		SYS_LOG_DBG("dispatching");
		return handle_req(data.rx);
	}
	return 0;
}

int set_idle_cb(struct usb_setup_packet *setup, s32_t *len, u8_t **data)
{
	SYS_LOG_DBG("Set Idle callback");

	/* TODO: Do something */

	return 0;
}

int get_report_cb(struct usb_setup_packet *setup, s32_t *len, u8_t **data)
{
	SYS_LOG_DBG("Get report callback");

	/* TODO: Do something */

	return 0;
}

static struct hid_ops ops = {
	.get_report = get_report_cb,
	.get_idle = debug_cb,
	.get_protocol = debug_cb,
	.set_report = set_report_cb,
	.set_idle = set_idle_cb,
	.set_protocol = debug_cb,
};

void main(void)
{
	SYS_LOG_DBG("Starting application");

	k_work_init(&data.tx_work, tx);

	usb_hid_register_device(hid_report_desc, sizeof(hid_report_desc),
				&ops);

	usb_hid_init();

	while (true) {
		k_sleep(K_SECONDS(2));
	}
}
