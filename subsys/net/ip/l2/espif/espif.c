/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_DOMAIN "espif"
#define NET_LOG_ENABLED 1
#include <stdio.h>

#include <kernel.h>

#include <console/uart_pipe.h>
#include <crc16.h>
#include <crc7.h>
#include <errno.h>
#include <misc/slist.h>
#include <misc/util.h>
#include <net/buf.h>
#include <net/net_core.h>
#include <net/net_if.h>
#include <net/net_offload.h>
#include <net/net_pkt.h>
#include <stdbool.h>
#include <stddef.h>
#include <uart.h>

#include "espif_protocol.h"

#define ESPIF_ISR_POLL_TIME 200
#define ESPIF_SYNC_POLL_TIME 500
#define ESPIF_RECV_TIMEOUT 500

struct espif_net_context {
	sys_snode_t node;
	struct net_context *context;
	int fd;
};

struct espif_dev_context {
	struct k_mutex mut;
	struct k_delayed_work tick;
	struct device *dev;

	struct device *port;

	bool synced;

	u8_t mac_addr[6];
	sys_slist_t contexts;
};

static const struct net_offload espif_offload_api;

static int espif_dev_status_to_err(u8_t status)
{
	switch (status) {
	case RESPONSE_OK:
		return 0;

	case RESPONSE_UNSUPPORTED_COMMAND:
	case RESPONSE_GENERAL_ERROR:
		return -EINVAL;

	default:
		return -EIO;
	}
}

static bool espif_dev_is_fatal(int err) { return err == -EIO; }

static void espif_dev_flush(struct device *dev)
{
	struct espif_dev_context *ctx = dev->driver_data;
	u8_t ch;

	while (uart_poll_in(ctx->port, &ch) == 0) {
	}
}

static void espif_dev_send(struct device *dev, struct net_buf_simple *buf,
			   int *err)
{
	struct espif_dev_context *ctx = dev->driver_data;
	u8_t *p = buf->data;
	u8_t *end = buf->data + buf->len;

	if (*err != 0) {
		return;
	}

	for (; p != end; p++) {
		uart_poll_out(ctx->port, *p);
	}
}

static int espif_dev_recv(struct device *dev, struct net_buf_simple *buf,
			  int len, int *err)
{
	struct espif_dev_context *ctx = dev->driver_data;
	u8_t *p = net_buf_simple_add(buf, len);
	u8_t *end = p + len;
	u32_t start = k_uptime_get_32();

	if (*err != 0) {
		return *err;
	}

	for (; p != end;) {
		u8_t ch;

		if (uart_poll_in(ctx->port, &ch) == 0) {
			*p++ = ch;
		} else {
			s32_t elapsed = k_uptime_get_32() - start;

			if (elapsed >= ESPIF_RECV_TIMEOUT) {
				*err = -ETIMEDOUT;
				return *err;
			}
		}
	}

	return 0;
}

static int espif_dev_recv_data(struct device *dev, struct net_buf_simple *buf,
			       int len, int *err)
{
	struct espif_dev_context *ctx = dev->driver_data;
	u8_t id;

	net_buf_simple_init(buf, 0);

	if (espif_dev_recv(dev, buf, 1 + len + 2, err) != 0) {
		goto erred;
	}

	if (crc_ccitt(0, buf->data, buf->len) != 0) {
		SYS_LOG_WRN("crc failure");
		*err = -EIO;
		goto erred;
	}

	id = net_buf_simple_pull_u8(buf);
	if ((id & ~COMMAND_MASK) != DATA_START) {
		SYS_LOG_WRN("invalid data marker id=%d", id);
		*err = -EIO;
		goto erred;
	}

	return *err;

erred:
	if (espif_dev_is_fatal(*err)) {
		ctx->synced = false;
	}

	return *err;
}

static int espif_dev_send_cmd(struct device *dev, struct net_buf_simple *buf,
			      int *err)
{
	struct espif_dev_context *ctx = dev->driver_data;
	u8_t cmd_id;
	u8_t resp;
	u8_t status;

	if (!ctx->synced) {
		*err = -EIO;
		goto erred;
	}
	if (*err != 0) {
		goto erred;
	}

	net_buf_simple_add_u8(buf, crc7(0, buf->data, buf->len) << 1);

	espif_dev_send(dev, buf, err);
	if (espif_dev_recv(dev, buf, 2, err) != 0) {
		goto erred;
	}

	cmd_id = buf->data[0];
	resp = buf->data[buf->len - 2];

	if (cmd_id != resp) {
		SYS_LOG_DBG("expected cmd_id=%u, got %u", cmd_id, resp);
		*err = -EIO;
		goto erred;
	}

	status = buf->data[buf->len - 1];
	*err = espif_dev_status_to_err(status);

	if (*err != 0) {
		SYS_LOG_DBG("remote returned error status=%u", status);
		goto erred;
	}

	return *err;

erred:
	if (espif_dev_is_fatal(*err)) {
		ctx->synced = false;
	}

	return *err;
}

static u32_t espif_dev_read_u32(struct device *dev, u32_t addr, int *err)
{
	struct net_buf_simple *buf = NET_BUF_SIMPLE(10);

	SYS_LOG_DBG("addr=%d", addr);

	net_buf_simple_init(buf, 0);

	net_buf_simple_add_u8(buf, COMMAND_SINGLE_READ);
	net_buf_simple_add_be24(buf, addr);

	espif_dev_send_cmd(dev, buf, err);

	if (espif_dev_recv_data(dev, buf, sizeof(u32_t), err) != 0) {
		return 0;
	}

	return net_buf_simple_pull_be32(buf);
}

static int espif_dev_write_u32(struct device *dev, int addr, u32_t value,
			       int *err)
{
	struct net_buf_simple *buf = NET_BUF_SIMPLE(10);

	SYS_LOG_DBG("addr=%d value=%d", addr, value);
	net_buf_simple_init(buf, 0);

	net_buf_simple_add_u8(buf, COMMAND_SINGLE_WRITE);
	net_buf_simple_add_be24(buf, addr);
	net_buf_simple_add_be32(buf, value);

	return espif_dev_send_cmd(dev, buf, err);
}

static void espif_dev_next(struct device *dev)
{
	struct espif_dev_context *ctx = dev->driver_data;

	k_mutex_lock(&ctx->mut, K_FOREVER);
	k_delayed_work_submit(&ctx->tick,
			      ctx->synced ? ESPIF_ISR_POLL_TIME
					  : ESPIF_SYNC_POLL_TIME);

	k_mutex_unlock(&ctx->mut);
}

static void espif_dev_tick(struct k_work *work)
{
	struct espif_dev_context *ctx =
	    CONTAINER_OF(work, struct espif_dev_context, tick);
	struct device *dev = ctx->dev;
	int err = 0;

	k_mutex_lock(&ctx->mut, K_FOREVER);

	if (!ctx->synced) {
		u32_t protocol;

		espif_dev_flush(dev);
		ctx->synced = true;

		protocol = espif_dev_read_u32(dev, API_LINK_PROTOCOL, &err);

		if (err == 0 && protocol == 0x100) {
			ctx->synced = true;
		}
	} else {
		u32_t event_fds =
		    espif_dev_read_u32(dev, API_SOCKETS_EVENT_FDS, &err);

		SYS_LOG_WRN("uptime=%u",
			    espif_dev_read_u32(dev, API_SYSTEM_UPTIME, &err));

		if (event_fds != 0) {
			u32_t fd = find_lsb_set(event_fds);
			u32_t events = espif_dev_read_u32(
			    dev, API_SOCKET_EVENTS + fd * API_SOCKET_STRIDE,
			    &err);

			SYS_LOG_DBG("fd=%d events=%x", fd, events);

			espif_dev_write_u32(
			    dev, API_SOCKET_EVENTS + fd * API_SOCKET_STRIDE,
			    events, &err);
		}
	}

	k_mutex_unlock(&ctx->mut);

	espif_dev_next(dev);
}

static int espif_dev_init(struct device *dev)
{
	struct espif_dev_context *ctx = dev->driver_data;

	ctx->dev = dev;
	ctx->port = device_get_binding(CONFIG_UART_PIPE_ON_DEV_NAME);

	k_mutex_init(&ctx->mut);
	k_delayed_work_init(&ctx->tick, espif_dev_tick);

	ctx->synced = false;

	espif_dev_next(dev);

	return 0;
}

static int espif_if_send(struct net_if *iface, struct net_pkt *pkt)
{
	NET_DBG("");

	return 0;
}

static int espif_get(sa_family_t family, enum net_sock_type type,
		     enum net_ip_protocol ip_proto,
		     struct net_context **context)
{
	NET_DBG("");

	return 0;
}

static int espif_bind(struct net_context *context, const struct sockaddr *addr,
		      socklen_t addrlen)
{
	const struct sockaddr_in *addr4 = (const struct sockaddr_in *)addr;

	NET_DBG("port=%u", addr4->sin_port);

	return 0;
}

static int espif_listen(struct net_context *context, int backlog)
{
	NET_DBG("");
	return -1;
}

static int espif_connect(struct net_context *context,
			 const struct sockaddr *addr, socklen_t addrlen,
			 net_context_connect_cb_t cb, s32_t timeout,
			 void *user_data)
{
	NET_DBG("");

	return 0;
}

static int espif_accept(struct net_context *context, net_tcp_accept_cb_t cb,
			s32_t timeout, void *user_data)
{
	NET_DBG("");
	return -1;
}

static int espif_send(struct net_pkt *pkt, net_context_send_cb_t cb,
		      s32_t timeout, void *token, void *user_data)
{
	NET_DBG("");
	return -1;
}

static int espif_sendto(struct net_pkt *pkt, const struct sockaddr *dst_addr,
			socklen_t addrlen, net_context_send_cb_t cb,
			s32_t timeout, void *token, void *user_data)
{
	NET_DBG("");
	return -1;
}

static int espif_recv(struct net_context *context, net_context_recv_cb_t cb,
		      s32_t timeout, void *user_data)
{
	NET_DBG("");
	return -1;
}

static int espif_put(struct net_context *context)
{
	NET_DBG("");
	return -1;
}

static void espif_if_init(struct net_if *iface)
{
	struct espif_dev_context *ctx = net_if_get_device(iface)->driver_data;

	NET_DBG("");

	iface->offload = &espif_offload_api;

	ctx->mac_addr[0] = 0x00;
	ctx->mac_addr[1] = 0x00;
	ctx->mac_addr[2] = 0x5E;
	ctx->mac_addr[3] = 0x00;
	ctx->mac_addr[4] = 0x53;
	ctx->mac_addr[5] = sys_rand32_get();

	net_if_set_link_addr(iface, ctx->mac_addr, sizeof(ctx->mac_addr),
			     NET_LINK_ETHERNET);
}

static const struct net_offload espif_offload_api = {
	.get = espif_get,
	.bind = espif_bind,
	.listen = espif_listen,
	.connect = espif_connect,
	.accept = espif_accept,
	.send = espif_send,
	.sendto = espif_sendto,
	.recv = espif_recv,
	.put = espif_put,
};

static struct net_if_api espif_if_api = {
	.init = espif_if_init, .send = espif_if_send,
};

static struct espif_dev_context espif_dev_context_0;

NET_DEVICE_INIT(espif, "espif0", espif_dev_init, &espif_dev_context_0, NULL,
		CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &espif_if_api,
		OFFLOAD_IP_L2, OFFLOAD_IP_L2_CTX_TYPE, 576);
