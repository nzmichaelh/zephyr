/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * AHDLC driver using uart_pipe.
 */

#define SYS_LOG_DOMAIN "ahdlc"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_AHDLC_LEVEL
#include <logging/sys_log.h>
#include <stdio.h>

#include <kernel.h>

#include <console/uart_pipe.h>
#include <errno.h>
#include <misc/util.h>
#include <net/buf.h>
#include <net/net_core.h>
#include <net/net_if.h>
#include <net/net_pkt.h>
#include <stdbool.h>
#include <stddef.h>

#define AHDLC_FLAG 0x7e
#define AHDLC_ESC 0x7d
#define AHDLC_XOR 0x20

#define AHDLC_INIT_FCS 0xffff
#define AHDLC_FCS_XOR 0xffff
#define AHDLC_GOOD_FCS 0xf0b8

enum ahdlc_state {
	STATE_INVALID,
	STATE_GARBAGE,
	STATE_SOF,
	STATE_IN_FRAME,
	STATE_ESC,
};

struct ahdlc_context {
	struct net_if *iface;
	struct net_pkt *rx;   /* and then placed into this net_pkt */
	struct net_buf *last; /* Pointer to last fragment in the list */
	u8_t *ptr;	    /* Where in net_pkt to add data */
	u8_t buf[1];	  /* AHDLC data is read into this buf */
	u8_t state;
	u16_t rx_fcs;

	u8_t mac_addr[6];
	struct net_linkaddr ll_addr;
        
#if defined(CONFIG_AHDLC_STATISTICS)
	u16_t frames;
	u16_t garbage;
	u16_t bad_fcs;
#define AHDLC_INC(var) printk("%s: %u\n", #var, ++ahdlc->var)
#else
#define AHDLC_INC(var)
#endif
};

static inline void ahdlc_writeb(unsigned char c)
{
	uart_pipe_send(&c, sizeof(c));
}

/**
 *  @brief Write byte to AHDLC, escape if it is END or ESC character
 *
 *  @param c  a byte to write
 */
static void ahdlc_writeb_esc(unsigned char c)
{
	static const u8_t flag[] = {AHDLC_ESC, AHDLC_FLAG ^ AHDLC_XOR};
	static const u8_t esc[] = {AHDLC_ESC, AHDLC_ESC ^ AHDLC_XOR};

	switch (c) {
	case AHDLC_FLAG:
		uart_pipe_send(flag, sizeof(flag));
		break;
	case AHDLC_ESC:
		uart_pipe_send(esc, sizeof(esc));
		break;
	default:
		uart_pipe_send(&c, sizeof(c));
	}
}

static u16_t ahdlc_fcs(u16_t seed, u8_t ch)
{
	u16_t b;

	b = (seed ^ ch) & 0xFF;
	b = (b ^ (b << 4) & 0xFF);
	b = (b << 8) ^ (b << 3) ^ (b >> 4);

	return ((seed >> 8) ^ b);
}

static int ahdlc_send(struct net_if *iface, struct net_pkt *pkt)
{
	struct net_buf *frag;
	u8_t *ptr;
	u16_t i;
	u16_t fcs = AHDLC_INIT_FCS;

	SYS_LOG_DBG("[%p]", iface);

	if (!pkt->frags) {
		/* No data? */
		return -ENODATA;
	}

	for (frag = pkt->frags; frag; frag = frag->frags) {
		SYS_LOG_DBG("%p %d [%p]", frag, frag->len, iface);
		ptr = frag->data;

		for (i = 0; i < frag->len; ++i) {
			u8_t ch = *ptr++;
			ahdlc_writeb_esc(ch);
			fcs = ahdlc_fcs(fcs, ch);
		}
	}

	net_pkt_unref(pkt);

	fcs ^= AHDLC_FCS_XOR;
	ahdlc_writeb_esc((u8_t)(fcs >> 0));
	ahdlc_writeb_esc((u8_t)(fcs >> 8));
	ahdlc_writeb(AHDLC_FLAG);

	return 0;
}

static struct net_pkt *ahdlc_poll_handler(struct ahdlc_context *ahdlc)
{
	SYS_LOG_DBG("[%p]", ahdlc);

	if (ahdlc->last && ahdlc->last->len) {
		return ahdlc->rx;
	}

	return NULL;
}

static void ahdlc_process_msg(struct ahdlc_context *ahdlc)
{
	struct net_pkt *pkt;

	SYS_LOG_DBG("[%p]", ahdlc);

	pkt = ahdlc_poll_handler(ahdlc);
	if (!pkt || !pkt->frags) {
		return;
	}

	if (net_recv_data(ahdlc->iface, pkt) < 0) {
		net_pkt_unref(pkt);
	}

	ahdlc->rx = NULL;
	ahdlc->last = NULL;
}

static inline int ahdlc_input_byte(struct ahdlc_context *ahdlc, u8_t ch)
{
	SYS_LOG_DBG("%u %x [%p]", ahdlc->state, ch, ahdlc);

	if (ahdlc->state == STATE_INVALID) {
		/* Not initialized */
		return 0;
	} else if (ch == AHDLC_FLAG) {
		ahdlc->state = STATE_SOF;

		if (ahdlc->rx == NULL) {
			return 0;
		} else if (ahdlc->rx_fcs != AHDLC_GOOD_FCS) {
                        SYS_LOG_ERR("[%p] bad_fcs %x", ahdlc, ahdlc->rx_fcs);
			AHDLC_INC(bad_fcs);
			net_pkt_unref(ahdlc->rx);
			ahdlc->rx = NULL;
			return 0;
		} else {
			AHDLC_INC(frames);
			return 1;
		}
	} else if (ahdlc->state == STATE_GARBAGE) {
		/* Drop */
		AHDLC_INC(garbage);
		return 0;
	} else {
		if (ahdlc->state == STATE_SOF) {
			ahdlc->rx_fcs = AHDLC_INIT_FCS;
			ahdlc->rx = net_pkt_get_reserve_rx(0, K_NO_WAIT);
			if (!ahdlc->rx) {
				SYS_LOG_ERR("[%p] cannot allocate pkt", ahdlc);
				goto err;
			}

			ahdlc->last = net_pkt_get_frag(ahdlc->rx, K_NO_WAIT);
			if (!ahdlc->last) {
				SYS_LOG_ERR(
				    "[%p] cannot allocate 1st data frag",
				    ahdlc);
				goto err;
			}

			net_pkt_frag_add(ahdlc->rx, ahdlc->last);
			ahdlc->ptr = net_pkt_ip_data(ahdlc->rx);
		}

                if (ch == AHDLC_ESC) {
                        ahdlc->state = STATE_ESC;
                } else {
                        if (ahdlc->state == STATE_ESC) {
                                ch ^= AHDLC_XOR;
                        }
                        ahdlc->state = STATE_IN_FRAME;

                        if (!net_buf_tailroom(ahdlc->last)) {
                                /* We need to allocate a new fragment */
                                struct net_buf *frag;

                                frag = net_pkt_get_reserve_rx_data(0, K_NO_WAIT);
                                if (!frag) {
                                        SYS_LOG_ERR(
                                                "[%p] cannot allocate next data frag",
                                                ahdlc);
                                        goto err;
                                }
                                net_buf_frag_insert(ahdlc->last, frag);
                                ahdlc->last = frag;
                                ahdlc->ptr = ahdlc->last->data;
                        }
                        ahdlc->ptr = net_buf_add_u8(ahdlc->last, ch);
                        ahdlc->ptr++;
                        ahdlc->rx_fcs = ahdlc_fcs(ahdlc->rx_fcs, ch);
                }
                return 0;
        }

err:
	if (ahdlc->rx != NULL) {
		net_pkt_unref(ahdlc->rx);
		ahdlc->rx = NULL;
	}
	ahdlc->last = NULL;
	return -1;
}

static u8_t *ahdlc_recv_cb(u8_t *buf, size_t *off)
{
	struct ahdlc_context *ahdlc =
	    CONTAINER_OF(buf, struct ahdlc_context, buf);
	size_t i;

	for (i = 0; i < *off; i++) {
		int err = ahdlc_input_byte(ahdlc, buf[i]);

		if (err > 0) {
			ahdlc_process_msg(ahdlc);
		} else if (err < 0) {
			/* Decode error, drop the rest. */
			break;
		}
	}

	*off = 0;

	return buf;
}

static int ahdlc_init(struct device *dev)
{
	struct ahdlc_context *ahdlc = dev->driver_data;

	SYS_LOG_DBG("[%p] dev %p", ahdlc, dev);

	ahdlc->state = STATE_INVALID;
	ahdlc->rx = NULL;
	ahdlc->last = NULL;

	return 0;
}

static inline struct net_linkaddr *ahdlc_get_mac(struct ahdlc_context *ahdlc)
{
	ahdlc->ll_addr.addr = ahdlc->mac_addr;
	ahdlc->ll_addr.len = sizeof(ahdlc->mac_addr);

	return &ahdlc->ll_addr;
}

static void ahdlc_iface_init(struct net_if *iface)
{
	struct ahdlc_context *ahdlc = net_if_get_device(iface)->driver_data;
	struct net_linkaddr *ll_addr = ahdlc_get_mac(ahdlc);

	SYS_LOG_DBG("[%p]", ahdlc);

	ahdlc->state = STATE_GARBAGE;
	ahdlc->iface = iface;

	uart_pipe_register(ahdlc->buf, sizeof(ahdlc->buf), ahdlc_recv_cb);

        /* 00-00-5E-00-53-xx Documentation RFC 7042 */
        ahdlc->mac_addr[0] = 0x00;
        ahdlc->mac_addr[1] = 0x00;
        ahdlc->mac_addr[2] = 0x5E;
        ahdlc->mac_addr[3] = 0x00;
        ahdlc->mac_addr[4] = 0x53;
        ahdlc->mac_addr[5] = sys_rand32_get();
	net_if_set_link_addr(iface, ll_addr->addr, ll_addr->len,
			     NET_LINK_ETHERNET);
}

static struct net_if_api ahdlc_if_api = {
    .init = ahdlc_iface_init, .send = ahdlc_send,
};

static struct ahdlc_context ahdlc_context_data;

#define _AHDLC_L2_LAYER DUMMY_L2
#define _AHDLC_L2_CTX_TYPE NET_L2_GET_CTX_TYPE(DUMMY_L2)
#define _AHDLC_MTU 576

NET_DEVICE_INIT(ahdlc, CONFIG_AHDLC_DRV_NAME, ahdlc_init, &ahdlc_context_data,
		NULL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &ahdlc_if_api,
		_AHDLC_L2_LAYER, _AHDLC_L2_CTX_TYPE, _AHDLC_MTU);
