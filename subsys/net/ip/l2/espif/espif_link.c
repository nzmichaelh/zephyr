/*
 * Copyright (c) 2017 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SYS_LOG_DOMAIN "espif/link"
#define NET_LOG_ENABLED 1
#define NET_SYS_LOG_LEVEL SYS_LOG_LEVEL_INFO

#include <kernel.h>

#include <crc16.h>
#include <crc7.h>
#include <errno.h>
#include <misc/slist.h>
#include <misc/util.h>
#include <net/buf.h>
#include <net/net_core.h>
#include <stdbool.h>
#include <stddef.h>
#include <uart.h>

#include "espif_protocol.h"

#define ESPIF_RECV_TIMEOUT 500

struct espif_link_data {
	struct device *port;
	bool synced;
};

static int espif_link_status_to_err(u8_t status)
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

static bool espif_link_is_fatal(int err) { return err == -EIO; }

void espif_link_flush(struct device *dev)
{
	struct espif_link_data *data = dev->driver_data;
	u8_t ch;

	while (uart_poll_in(data->port, &ch) == 0) {
	}

	data->synced = true;
}

static void espif_link_send(struct device *dev, struct net_buf_simple *buf,
			    int *err)
{
	struct espif_link_data *data = dev->driver_data;
	u8_t *p = buf->data;
	u8_t *end = buf->data + buf->len;

	NET_DBG("%p: len=%d err=%d", dev, buf->len, *err);

	if (*err != 0) {
		return;
	}

	for (; p != end; p++) {
		uart_poll_out(data->port, *p);
	}
}

static int espif_link_recv(struct device *dev, struct net_buf_simple *buf,
			   int len, int *err)
{
	struct espif_link_data *data = dev->driver_data;
	u8_t *p = net_buf_simple_add(buf, len);
	u8_t *end = p + len;
	u32_t start = k_uptime_get_32();

	SYS_LOG_DBG("%p: len=%d err=%d", dev, len, *err);

	if (*err != 0) {
		return *err;
	}

	for (; p != end;) {
		u8_t ch;

		if (uart_poll_in(data->port, &ch) == 0) {
			*p++ = ch;
		} else {
			s32_t elapsed = k_uptime_get_32() - start;

			if (elapsed >= ESPIF_RECV_TIMEOUT) {
				SYS_LOG_DBG("%p: timeout after %d", dev,
					    len - (end - p));
				*err = -ETIMEDOUT;
				return *err;
			}
		}
	}

	return 0;
}

static int espif_link_recv_data(struct device *dev, struct net_buf_simple *buf,
				int len, int *err)
{
	struct espif_link_data *data = dev->driver_data;
	u8_t id;

	net_buf_simple_init(buf, 0);

	if (espif_link_recv(dev, buf, 1 + len + 2, err) != 0) {
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
	if (espif_link_is_fatal(*err)) {
		data->synced = false;
	}

	return *err;
}

static int espif_link_send_data(struct device *dev, struct net_buf_simple *src,
				int *err)
{
	struct espif_link_data *data = dev->driver_data;
	struct net_buf_simple *buf = NET_BUF_SIMPLE(10);
	u16_t crc;
	u8_t cmd_id;
	u8_t status;

	net_buf_simple_init(buf, 0);
	net_buf_simple_add_u8(buf, DATA_START | DATA_FIRST_PACKET);

	espif_link_send(dev, buf, err);
	espif_link_send(dev, src, err);

	crc = crc_ccitt(0, buf->data, buf->len);
	crc = crc_ccitt(crc, src->data, src->len);

	net_buf_simple_init(buf, 0);
	net_buf_simple_add_le16(buf, crc);

	espif_link_send(dev, buf, err);

	net_buf_simple_init(buf, 0);
	if (espif_link_recv(dev, buf, 2, err) != 0) {
		goto erred;
	}

	cmd_id = buf->data[0];
	status = buf->data[buf->len - 1];

	NET_DBG("cmd_id=%d status=%d", cmd_id, status);
	*err = espif_link_status_to_err(status);

	return *err;

erred:
	if (espif_link_is_fatal(*err)) {
		data->synced = false;
	}

	return *err;
}

int espif_link_send_cmd(struct device *dev, struct net_buf_simple *buf,
			int *err)
{
	struct espif_link_data *data = dev->driver_data;
	u8_t cmd_id;
	u8_t resp;
	u8_t status;

	if (!data->synced) {
		*err = -EIO;
		goto erred;
	}
	if (*err != 0) {
		goto erred;
	}

	net_buf_simple_add_u8(buf, crc7(0, buf->data, buf->len) << 1);

	espif_link_send(dev, buf, err);
	if (espif_link_recv(dev, buf, 2, err) != 0) {
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
	*err = espif_link_status_to_err(status);

	if (*err != 0) {
		SYS_LOG_DBG("remote returned error status=%u", status);
		goto erred;
	}

	return *err;

erred:
	if (espif_link_is_fatal(*err)) {
		data->synced = false;
	}

	return *err;
}

u32_t espif_link_read_u32(struct device *dev, u32_t addr, int *err)
{
	struct net_buf_simple *buf = NET_BUF_SIMPLE(10);

	SYS_LOG_DBG("addr=%d", addr);

	net_buf_simple_init(buf, 0);

	net_buf_simple_add_u8(buf, COMMAND_SINGLE_READ);
	net_buf_simple_add_be24(buf, addr);

	espif_link_send_cmd(dev, buf, err);

	if (espif_link_recv_data(dev, buf, sizeof(u32_t), err) != 0) {
		return 0;
	}

	return net_buf_simple_pull_be32(buf);
}

int espif_link_read_dma(struct device *dev, int addr,
			struct net_buf_simple *sink, int len, int *err)
{
	struct net_buf_simple *buf = NET_BUF_SIMPLE(10);

	SYS_LOG_DBG("addr=%d len=%d", addr, len);
	net_buf_simple_init(buf, 0);

	net_buf_simple_add_u8(buf, COMMAND_DMA_READ);
	net_buf_simple_add_be24(buf, addr);
	net_buf_simple_add_be16(buf, len);

	if (espif_link_send_cmd(dev, buf, err) != 0) {
		return *err;
	}

	return espif_link_recv_data(dev, sink, len, err);
}

int espif_link_write_u32(struct device *dev, int addr, u32_t value, int *err)
{
	struct net_buf_simple *buf = NET_BUF_SIMPLE(10);

	SYS_LOG_DBG("addr=%d value=%d", addr, value);
	net_buf_simple_init(buf, 0);

	net_buf_simple_add_u8(buf, COMMAND_SINGLE_WRITE);
	net_buf_simple_add_be24(buf, addr);
	net_buf_simple_add_be32(buf, value);

	return espif_link_send_cmd(dev, buf, err);
}

int espif_link_write_dma(struct device *dev, int addr,
			 struct net_buf_simple *write, int *err)
{
	struct net_buf_simple *buf = NET_BUF_SIMPLE(10);

	SYS_LOG_DBG("addr=%d len=%d", addr, write->len);
	net_buf_simple_init(buf, 0);

	net_buf_simple_add_u8(buf, COMMAND_DMA_WRITE);
	net_buf_simple_add_be24(buf, addr);
	net_buf_simple_add_be16(buf, write->len);

	if (espif_link_send_cmd(dev, buf, err) != 0) {
		return *err;
	}

	return espif_link_send_data(dev, write, err);
}

void espif_link_drop_sync(struct device *dev)
{
	struct espif_link_data *data = dev->driver_data;

	data->synced = false;
}

bool espif_link_synced(struct device *dev)
{
	struct espif_link_data *data = dev->driver_data;

	return data->synced;
}

static int espif_link_init(struct device *dev)
{
	struct espif_link_data *data = dev->driver_data;

	data->port = device_get_binding(CONFIG_UART_PIPE_ON_DEV_NAME);
	data->synced = false;

	return 0;
}

struct espif_link_api {
};

static const struct espif_link_api espif_link_api_0;

static struct espif_link_data espif_link_data_0;

DEVICE_AND_API_INIT(esplink0, "esplink0", espif_link_init, &espif_link_data_0,
		    NULL, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &espif_link_api_0);
