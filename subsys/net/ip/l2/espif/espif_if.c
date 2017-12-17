/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_DOMAIN "espif/if"
#define NET_LOG_ENABLED 1
#define NET_SYS_LOG_LEVEL SYS_LOG_LEVEL_DEBUG

#include <kernel.h>

#include <errno.h>
#include <net/buf.h>
#include <net/net_core.h>
#include <net/net_if.h>
#include <net/net_offload.h>
#include <net/net_pkt.h>

#include "espif_link.h"
#include "espif_protocol.h"

#define ESPIF_ISR_POLL_TIME 200
#define ESPIF_SYNC_POLL_TIME 500

struct espif_net_context {
	struct net_context *context;
	int fd;
	net_context_connect_cb_t connect_cb;
	net_context_send_cb_t send_cb;
	net_context_send_cb_t sendto_cb;
	void *token;
	void *user_data;

	net_context_recv_cb_t recv_cb;
	void *recv_data;
};

struct espif_if_context {
	struct device *dev;
	struct device *link;

	struct k_mutex mut;
	struct k_delayed_work tick;

	u8_t mac_addr[6];

	struct espif_net_context net_ctx[CONFIG_NET_MAX_CONN];
	struct net_buf_simple buf;
	u8_t buf_data[1600];
};

static const struct net_offload espif_offload_api;

static void espif_if_next(struct device *dev)
{
	struct espif_if_context *ctx = dev->driver_data;

	k_mutex_lock(&ctx->mut, K_FOREVER);
	k_delayed_work_submit(&ctx->tick,
			      espif_link_synced(ctx->link)
			      ? ESPIF_ISR_POLL_TIME
			      : ESPIF_SYNC_POLL_TIME);

	k_mutex_unlock(&ctx->mut);
}

static int espif_if_dispatch_recv(struct device *dev,
				  struct espif_net_context *net_ctx, int *err)
{
	struct espif_if_context *ctx = dev->driver_data;
	struct net_pkt *pkt = NULL;
	s32_t avail;

	avail = espif_link_read_u32(ctx->link,
				    API_SOCKET_RECV_AVAIL + net_ctx->fd, err);

	if (avail < 0) {
		NET_WARN("avail=%d", avail);
		*err = avail;
	}

	if (*err != 0) {
		return *err;
	}

	pkt = net_pkt_get_rx(net_ctx->context, K_NO_WAIT);

	if (pkt == NULL) {
		NET_WARN("couldn't allocate a rx_pkt");
		goto enomem;
	}

	if (espif_link_read_dma(ctx->link, API_SOCKET_RECV + net_ctx->fd,
				&ctx->buf, avail, err) != 0) {
		NET_WARN("read error err=%d", *err);
		goto erred;
	}

	if (!net_pkt_append_all(pkt, avail, ctx->buf.data, K_NO_WAIT)) {
		NET_WARN("couldn't append data");
		goto enomem;
	}

	espif_link_write_u32(ctx->link, API_SOCKET_RECV_AVAIL + net_ctx->fd,
			     avail, err);

	if (net_ctx->recv_cb != NULL) {
		NET_INFO("dispatching read cb len=%d", avail);
		net_pkt_set_appdata(pkt, net_pkt_ip_data(pkt));
		net_pkt_set_appdatalen(pkt, net_pkt_get_len(pkt));
		net_ctx->recv_cb(net_ctx->context, pkt, 0, net_ctx->recv_data);
	} else {
		net_pkt_unref(pkt);
	}

	return *err;

enomem:
	*err = -ENOMEM;
erred:
	if (pkt != NULL) {
		net_pkt_unref(pkt);
	}

	return *err;
}

static void espif_if_dispatch_events(struct device *dev, u32_t event_fds,
				     int *err)
{
	struct espif_if_context *ctx = dev->driver_data;

	while (*err == 0 && event_fds != 0) {
		struct espif_net_context *net_ctx = NULL;
		u32_t fd_bit = find_lsb_set(event_fds) - 1;
		int fd = fd_bit * API_SOCKET_STRIDE;
		int i;

		event_fds &= ~(1 << fd_bit);

		for (i = 0; i < CONFIG_NET_MAX_CONN; i++) {
			if (ctx->net_ctx[i].fd == fd) {
				net_ctx = &ctx->net_ctx[i];
				break;
			}
		}

		u32_t events = espif_link_read_u32(
			ctx->link, API_SOCKET_EVENTS + fd, err);

		espif_link_write_u32(ctx->link, API_SOCKET_EVENTS + fd, events,
				     err);

		if (events == 0) {
			continue;
		}

		NET_INFO("err=%d fd=%d events=%x", *err, fd, events);

		if (*err != 0) {
		}
		if (net_ctx == NULL) {
			NET_WARN("action on a non-exisent fd");
			continue;
		}

		if ((events & SOCKET_EVENT_CONNECTED) != 0) {
			if (net_ctx->connect_cb) {
				NET_INFO("calling connect_cb");
				net_ctx->connect_cb(net_ctx->context, 0,
						    net_ctx->user_data);
			}
		}

		if ((events & SOCKET_EVENT_WRITE_READY) != 0) {
			if (net_ctx->send_cb) {
				NET_INFO("calling send_cb");
				net_ctx->send_cb(net_ctx->context, 0,
						 net_ctx->token,
						 net_ctx->user_data);
			}
		}

		if ((events & SOCKET_EVENT_READ_READY) != 0) {
			espif_if_dispatch_recv(dev, net_ctx, err);
		}
	}
}

static void espif_if_tick(struct k_work *work)
{
	struct espif_if_context *ctx =
		CONTAINER_OF(work, struct espif_if_context, tick);
	struct device *dev = ctx->dev;
	int err = 0;

	k_mutex_lock(&ctx->mut, K_FOREVER);

	if (!espif_link_synced(ctx->link)) {
		u32_t protocol;

		espif_link_flush(ctx->link);

		protocol =
			espif_link_read_u32(ctx->link, API_LINK_PROTOCOL, &err);

		if (protocol != VERSION_PROTOCOL) {
			NET_WARN("unrecognised protocol=%d", protocol);
			espif_link_drop_sync(ctx->link);
		}
	} else {
		espif_if_dispatch_events(
			dev,
			espif_link_read_u32(
				ctx->link, API_SOCKETS_EVENT_FDS, &err),
			&err);
	}

	k_mutex_unlock(&ctx->mut);

	espif_if_next(dev);
}

static int espif_if_dev_init(struct device *dev)
{
	struct espif_if_context *ctx = dev->driver_data;

	ctx->buf.data = ctx->buf_data;
	net_buf_simple_init(&ctx->buf, 0);

	ctx->dev = dev;
	ctx->link = device_get_binding("esplink0");

	k_mutex_init(&ctx->mut);
	k_delayed_work_init(&ctx->tick, espif_if_tick);

	espif_if_next(dev);

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
	struct net_if *iface = net_if_get_by_index((*context)->iface);
	struct device *dev = net_if_get_device(iface);
	struct espif_if_context *ctx = dev->driver_data;
	struct net_buf_simple *buf = NET_BUF_SIMPLE(10);
	int err = 0;
	int fd;
	int i;
	struct espif_net_context *net_ctx = NULL;

	NET_DBG("");

	net_buf_simple_init(buf, 0);
	net_buf_simple_add_u8(buf, family);
	net_buf_simple_add_u8(buf, type);
	net_buf_simple_add_u8(buf, ip_proto);

	k_mutex_lock(&ctx->mut, K_FOREVER);

	for (i = 0; i < CONFIG_NET_MAX_CONN; i++) {
		if (ctx->net_ctx[i].context == NULL) {
			net_ctx = &ctx->net_ctx[i];
			break;
		}
	}

	if (net_ctx == NULL) {
		NET_INFO("no free context");
		k_mutex_unlock(&ctx->mut);
		return -EMFILE;
	}

	(*context)->offload_data = net_ctx;

	net_ctx->fd = -1;
	net_ctx->connect_cb = NULL;

	espif_link_write_dma(ctx->link, API_SOCKETS_GET, buf, &err);
	fd = espif_link_read_u32(ctx->link, API_SOCKETS_ERR, &err);

	if (err != 0) {
		NET_INFO("remote returned protocol err=%d", err);
		k_mutex_unlock(&ctx->mut);
		return err;
	}

	if (fd < 0) {
		NET_INFO("remote returned err=%d", err);
		k_mutex_unlock(&ctx->mut);
		return fd;
	}

	net_ctx->context = *context;
	k_mutex_unlock(&ctx->mut);
	net_ctx->fd = fd;

	return 0;
}

static int espif_bind(struct net_context *context, const struct sockaddr *addr,
		      socklen_t addrlen)
{
	struct net_if *iface = net_if_get_by_index(context->iface);
	struct device *dev = net_if_get_device(iface);
	struct espif_if_context *ctx = dev->driver_data;
	struct espif_net_context *net_ctx = context->offload_data;
	struct net_buf_simple *buf = NET_BUF_SIMPLE(10);
	const struct sockaddr_in *addr4 = (const struct sockaddr_in *)addr;
	int err = 0;
	int port;

	NET_INFO("fd=%d port=%u", net_ctx->fd, addr4->sin_port);

	net_buf_simple_init(buf, 0);
	net_buf_simple_add_u8(buf, addr4->sin_family);
	net_buf_simple_add_be16(buf, addr4->sin_port);
	net_buf_simple_add_be32(buf, addr4->sin_addr.in4_u.u4_addr32[0]);

	k_mutex_lock(&ctx->mut, K_FOREVER);

	espif_link_write_dma(ctx->link, API_SOCKET_BIND + net_ctx->fd, buf,
			     &err);
	port = espif_link_read_u32(ctx->link, API_SOCKETS_ERR, &err);

	k_mutex_unlock(&ctx->mut);

	if (err != 0) {
		NET_INFO("err=%d", err);
	} else if (port < 0) {
		NET_INFO("remote returned port=%d", port);
		err = port;
	} else {
		/* TODO: copy port number. */
	}

	return err;
}

static int espif_listen(struct net_context *context, int backlog)
{
	NET_WARN("not implemented");

	return -1;
}

static int espif_connect(struct net_context *context,
			 const struct sockaddr *addr, socklen_t addrlen,
			 net_context_connect_cb_t cb, s32_t timeout,
			 void *user_data)
{
	struct net_if *iface = net_if_get_by_index(context->iface);
	struct device *dev = net_if_get_device(iface);
	struct espif_if_context *ctx = dev->driver_data;
	struct espif_net_context *net_ctx = context->offload_data;
	struct net_buf_simple *buf = NET_BUF_SIMPLE(10);
	const struct sockaddr_in *addr4 = (const struct sockaddr_in *)addr;
	int err = 0;

	NET_INFO("fd=%d family=%d addr=%x port=%d timeout=%d cb=%p",
		 net_ctx->fd, addr4->sin_family,
		 ntohl(addr4->sin_addr.in4_u.u4_addr32[0]),
		 ntohs(addr4->sin_port), timeout, cb);

	net_buf_simple_init(buf, 0);
	net_buf_simple_add_u8(buf, addr4->sin_family);
	net_buf_simple_add_be16(buf, ntohs(addr4->sin_port));
	net_buf_simple_add_be32(buf, ntohl(addr4->sin_addr.in4_u.u4_addr32[0]));

	k_mutex_lock(&ctx->mut, K_FOREVER);

	net_ctx->user_data = user_data;
	net_ctx->connect_cb = cb;

	espif_link_write_dma(ctx->link, API_SOCKET_CONNECT + net_ctx->fd, buf,
			     &err);
	k_mutex_unlock(&ctx->mut);

	if (err != 0) {
		NET_WARN("err=%d", err);
	}

	return err;
}

static int espif_accept(struct net_context *context, net_tcp_accept_cb_t cb,
			s32_t timeout, void *user_data)
{
	NET_WARN("not implemented");

	return -1;
}

static int espif_send(struct net_pkt *pkt, net_context_send_cb_t cb,
		      s32_t timeout, void *token, void *user_data)
{
	struct net_context *context = pkt->context;
	struct net_if *iface = net_if_get_by_index(context->iface);
	struct device *dev = net_if_get_device(iface);
	struct espif_if_context *ctx = dev->driver_data;
	struct espif_net_context *net_ctx = context->offload_data;
	int err = 0;

	NET_INFO("fd=%d len=%d timeout=%d cb=%p", net_ctx->fd,
		 net_pkt_get_len(pkt), timeout, cb);

	k_mutex_lock(&ctx->mut, K_FOREVER);

	net_ctx->token = token;
	net_ctx->user_data = user_data;
	net_ctx->send_cb = cb;

	espif_link_write_dma(ctx->link, API_SOCKET_SEND + net_ctx->fd,
			     &pkt->frags->b, &err);
	k_mutex_unlock(&ctx->mut);

	if (err != 0) {
		NET_WARN("err=%d", err);
	}

	return err;
}

static int espif_sendto(struct net_pkt *pkt, const struct sockaddr *dst_addr,
			socklen_t addrlen, net_context_send_cb_t cb,
			s32_t timeout, void *token, void *user_data)
{
	struct net_context *context = pkt->context;
	struct net_if *iface = net_if_get_by_index(context->iface);
	struct device *dev = net_if_get_device(iface);
	struct espif_if_context *ctx = dev->driver_data;
	struct espif_net_context *net_ctx = context->offload_data;
	const struct sockaddr_in *addr4 = (const struct sockaddr_in *)dst_addr;
	struct net_buf_simple *buf = NET_BUF_SIMPLE(10);
	int err = 0;

	NET_INFO("fd=%d ipaddr=%x port=%d len=%d timeout=%d cb=%p", net_ctx->fd,
		 addr4->sin_addr.in4_u.u4_addr32[0],
		 ntohs(addr4->sin_port),
		 net_pkt_get_len(pkt), timeout, cb);

	k_mutex_lock(&ctx->mut, K_FOREVER);

	net_buf_simple_init(buf, 0);
	net_buf_simple_add_u8(buf, addr4->sin_family);
	net_buf_simple_add_be16(buf, ntohs(addr4->sin_port));
	net_buf_simple_add_be32(buf, ntohl(addr4->sin_addr.in4_u.u4_addr32[0]));

	espif_link_write_dma(ctx->link,
			     API_SOCKET_SENDTO_DEST_ADDR + net_ctx->fd,
			     buf, &err);

	net_ctx->token = token;
	net_ctx->user_data = user_data;
	net_ctx->sendto_cb = cb;

	espif_link_write_dma(ctx->link, API_SOCKET_SENDTO + net_ctx->fd,
			     &pkt->frags->b, &err);

	k_mutex_unlock(&ctx->mut);

	if (err != 0) {
		NET_WARN("err=%d", err);
	}

	return err;
}

static int espif_recv(struct net_context *context, net_context_recv_cb_t cb,
		      s32_t timeout, void *user_data)
{
	struct net_if *iface = net_if_get_by_index(context->iface);
	struct device *dev = net_if_get_device(iface);
	struct espif_if_context *ctx = dev->driver_data;
	struct espif_net_context *net_ctx = context->offload_data;

	NET_INFO("fd=%d timeout=%d cb=%p", net_ctx->fd, timeout, cb);

	k_mutex_lock(&ctx->mut, K_FOREVER);

	net_ctx->recv_data = user_data;
	net_ctx->recv_cb = cb;

	k_mutex_unlock(&ctx->mut);

	return 0;
}

static int espif_put(struct net_context *context)
{
	struct net_if *iface = net_if_get_by_index(context->iface);
	struct device *dev = net_if_get_device(iface);
	struct espif_if_context *ctx = dev->driver_data;
	struct espif_net_context *net_ctx = context->offload_data;
	int err = 0;

	NET_INFO("fd=%d", net_ctx->fd);

	k_mutex_lock(&ctx->mut, K_FOREVER);
	espif_link_write_u32(ctx->link, API_SOCKET_PUT + net_ctx->fd, 0, &err);

	if (net_ctx->recv_cb != NULL) {
		net_ctx->recv_cb(context, NULL, 0, net_ctx->recv_data);
	}

	net_ctx->context = NULL;

	k_mutex_unlock(&ctx->mut);

	if (err != 0) {
		NET_WARN("err=%d", err);
	}

	return err;
}

static void espif_if_init(struct net_if *iface)
{
	struct espif_if_context *ctx = net_if_get_device(iface)->driver_data;

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

static struct espif_if_context espif_if_context_0;

NET_DEVICE_INIT(espif, "espif0", espif_if_dev_init, &espif_if_context_0, NULL,
		CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &espif_if_api,
		OFFLOAD_IP_L2, OFFLOAD_IP_L2_CTX_TYPE, 576);
