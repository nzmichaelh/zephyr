/*
 * Copyright (c) 2019 Tobias Svehagen
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT u_blox_nina

#define LOG_LEVEL CONFIG_WIFI_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(wifi_nina);

#include <kernel.h>
#include <ctype.h>
#include <errno.h>
#include <zephyr.h>
#include <device.h>
#include <init.h>
#include <stdlib.h>

#include <drivers/gpio.h>
#include <drivers/spi.h>

#include <net/net_if.h>
#include <net/net_offload.h>
#include <net/socket_offload.h>
#include <net/wifi_mgmt.h>

#include "nina_private.h"

struct nina_data {
	struct spi_config spi_config;
	struct spi_cs_control cs_ctrl;

	struct device *spi_dev;

	struct device *ready_dev;
	struct device *reset_dev;
	struct device *irq_dev;

	uint8_t ready_pin;
	uint8_t reset_pin;
	uint8_t irq_pin;
};

static const struct net_offload nina_offload;

#define DEV_DATA(dev) ((struct nina_data *)(dev)->driver_data)

struct nina_config {
};

struct nina_args {
	uint8_t buf[32];
};

struct nina_result {
	uint8_t buf[32];
};

void nina_args_init(struct nina_args *args)
{
}

void nina_args_addw(struct nina_args *args, uint32_t v)
{
}

void nina_args_addh(struct nina_args *args, uint16_t v)
{
}

void nina_args_addb(struct nina_args *args, uint8_t v)
{
}

int nina_sendrecv(struct nina_data *n, enum nina_cmd cmd,
		  struct nina_args *args, struct nina_result *res)
{
	LOG_INF("nina_sendrecv");
}

static int nina_wait_ready(struct nina_data *n)
{
	LOG_INF("nina_wait_ready");
	while (gpio_pin_get(n->ready_dev, n->ready_pin) != 0) {
		k_yield();
	}
	return 0;
}

static int nina_sync(struct nina_data *n)
{
	LOG_INF("nina_sync");
	return -EINVAL;
}

static void nina_iface_init(struct net_if *iface)
{
	LOG_INF("nina_iface_init");

	struct device *dev = net_if_get_device(iface);
	struct nina_data *data = dev->driver_data;

	iface->if_dev->offload = &nina_offload;
	LOG_INF("done");
}

static int nina_iface_scan(struct device *dev, scan_result_cb_t cb)
{
	LOG_INF("nina_iface_scan");
	return -EINVAL;
}

static int nina_iface_connect(struct device *dev,
			      struct wifi_connect_req_params *params)
{
	LOG_INF("nina_iface_connect");
	return -EINVAL;
}

static int nina_iface_disconnect(struct device *dev)
{
	LOG_INF("nina_iface_disconnect");
	return -EINVAL;
}

static int nina_iface_ap_enable(struct device *dev,
				struct wifi_connect_req_params *params)
{
	LOG_INF("nina_iface_ap_enable");
	return -EINVAL;
}

static int nina_iface_ap_disable(struct device *dev)
{
	LOG_INF("nina_iface_ap_disable");
	return -EINVAL;
}

struct net_wifi_mgmt_offload nina_mgmt_api = {
	.iface_api.init = nina_iface_init,
	.scan = nina_iface_scan,
	.connect = nina_iface_connect,
	.disconnect = nina_iface_disconnect,
	.ap_enable = nina_iface_ap_enable,
	.ap_disable = nina_iface_ap_disable,
};

static int nina_getaddrinfo(const char *node, const char *service,
			    const struct zsock_addrinfo *hints,
			    struct zsock_addrinfo **res)
{
	LOG_INF("nina_getaddrinfo");
	return -EINVAL;
}

static void nina_freeaddrinfo(struct zsock_addrinfo *res)
{
	LOG_INF("nina_freeaddrinfo");
}

static const struct socket_dns_offload nina_dns_api = {
	.getaddrinfo = nina_getaddrinfo,
	.freeaddrinfo = nina_freeaddrinfo,
};

static int nina_get(sa_family_t family, enum net_sock_type type,
		    enum net_ip_protocol ip_proto, struct net_context **context)
{
	LOG_INF("nina_get");
	struct nina_args args;
	struct nina_result res;

	nina_args_init(&args);
	nina_sendrecv(NULL, NINA_CMD_GET_SOCKET, &args, &res);
	return -EINVAL;
}

static int nina_bind(struct net_context *context, const struct sockaddr *addr,
		     socklen_t addrlen)
{
	LOG_INF("nina_bind");
	return -EINVAL;
}

static int nina_listen(struct net_context *context, int backlog)
{
	LOG_INF("nina_listen");
	struct nina_args args;
	struct nina_result res;

	nina_args_init(&args);
	/* nina_args_addw(&args, addr); */
	/* nina_args_addh(&args, port); */
	/* nina_args_addb(&args, socket); */
	/* nina_args_addb(&args, type); */

	nina_sendrecv(NULL, NINA_CMD_START_SERVER_TCP, &args, &res);
	return -EINVAL;
}

static int nina_connect(struct net_context *context,
			const struct sockaddr *addr, socklen_t addrlen,
			net_context_connect_cb_t cb, int32_t timeout,
			void *user_data)
{
	LOG_INF("nina_connect");
	struct nina_args args;
	struct nina_result res;

	nina_args_init(&args);
	/* nina_args_addw(&args, addr); */
	/* nina_args_addh(&args, port); */
	/* nina_args_addb(&args, socket); */
	/* nina_args_addb(&args, type); */
	nina_sendrecv(NULL, NINA_CMD_START_CLIENT_TCP, &args, &res);
	return -EINVAL;
}

static int nina_accept(struct net_context *context, net_tcp_accept_cb_t cb,
		       int32_t timeout, void *user_data)
{
	LOG_INF("nina_accept");
	return -EINVAL;
}

static int nina_send(struct net_pkt *pkt, net_context_send_cb_t cb,
		     int32_t timeout, void *user_data)
{
	LOG_INF("nina_send");
	return -EINVAL;
}

static int nina_sendto(struct net_pkt *pkt, const struct sockaddr *dst_addr,
		       socklen_t addrlen, net_context_send_cb_t cb,
		       int32_t timeout, void *user_data)
{
	LOG_INF("nina_sendto");
	return -EINVAL;
}

static int nina_recv(struct net_context *context, net_context_recv_cb_t cb,
		     int32_t timeout, void *user_data)
{
	LOG_INF("nina_recv");
	return -EINVAL;
}

static int nina_put(struct net_context *context)
{
	LOG_INF("nina_put");
	return -EINVAL;
}

static const struct net_offload nina_offload = {
	.get = nina_get,
	.bind = nina_bind,
	.listen = nina_listen,
	.connect = nina_connect,
	.accept = nina_accept,
	.send = nina_send,
	.sendto = nina_sendto,
	.recv = nina_recv,
	.put = nina_put,
};

static int nina_init(struct device *dev)
{
	LOG_INF("nina_init");

	struct nina_data *data = DEV_DATA(dev);

	data->spi_dev = device_get_binding(DT_INST_BUS_LABEL(0));
	if (!data->spi_dev) {
		return -EPERM;
	}

	data->spi_config = (struct spi_config){
		.frequency = DT_INST_PROP(0, spi_max_frequency),
		.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8),
		.slave = DT_INST_REG_ADDR(0),
	};
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	data->cs_ctrl = (struct spi_cs_control)
	{
		.gpio_dev =
		device_get_binding(DT_INST_SPI_DEV_CS_GPIOS_LABEL(0)),
		.gpio_pin = DT_INST_SPI_DEV_CS_GPIOS_PIN(0),
		.gpio_dt_flags = DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0),
	};
	if (data->cs_ctrl.gpio_dev == NULL) {
		return -EPERM;
	}
	data->spi_config.cs = &data->cs_ctrl;
#endif

	data->ready_dev =
		device_get_binding(DT_INST_GPIO_LABEL(0, ready_gpios));
	if (data->ready_dev == NULL) {
		return -EPERM;
	}
	data->ready_pin = DT_INST_GPIO_PIN(0, ready_gpios);

	data->irq_dev = device_get_binding(DT_INST_GPIO_LABEL(0, irq_gpios));
	if (data->irq_dev == NULL) {
		return -EPERM;
	}
	data->irq_pin = DT_INST_GPIO_PIN(0, irq_gpios);

	/* Reset is optional */
	data->reset_dev =
		device_get_binding(DT_INST_GPIO_LABEL(0, reset_gpios));
	data->reset_pin = DT_INST_GPIO_PIN(0, reset_gpios);

	LOG_INF("ok");
	return 0;
}

#define NINA_MTU 2048

static struct nina_data nina_data_0;

NET_DEVICE_OFFLOAD_INIT(wifi_nina, DT_INST_LABEL(0), nina_init,
			device_pm_control_nop, &nina_data_0, NULL,
			CONFIG_WIFI_INIT_PRIORITY, &nina_mgmt_api,
			NINA_MTU);
