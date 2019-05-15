/*
 * Copyright (c) 2019 Nikos Oikonomou <nikoikonomou92@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <stdio.h>
#include <net/net_core.h>
#include <device.h>
#include <net/ieee802154_radio.h>
#include <net/net_pkt.h>
#include <net/net_l2.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(s2868a1_sample, LOG_LEVEL_DBG);

static uint8_t data_buffer[255];

static void send_char(const struct device *dev, char c, uint16_t len)
{
	const struct ieee802154_radio_api *api = dev->api;
	struct net_buf frag = {
		.data = data_buffer,
		.len = len
	};

	for (int i = 0; i < len; i++) {
		data_buffer[i] = c;
	}
	api->tx(dev, IEEE802154_TX_MODE_DIRECT, NULL, &frag);
}

void main(void)
{
	const struct device *dev = device_get_binding(
		CONFIG_NET_CONFIG_IEEE802154_DEV_NAME);
	const struct ieee802154_radio_api *api = dev->api;

	if (dev == NULL) {
		LOG_ERR("Device not found");
		return;
	}
	LOG_INF("S2868A1 Sample started");
	api->start(dev);
	for (int i = 0; i < 100; i++) {
		send_char(dev, 'a' + i % 5, 25 + i);
	}
	for (int i = 0; i < 100; i++) {
		send_char(dev, 'e' - i % 5, 124 - i);
	}
	while (1) {
		k_sleep(K_SECONDS(1));
	}
}

int net_recv_data(struct net_if *iface, struct net_pkt *pkt)
{
	uint8_t *data = net_pkt_data(pkt);

	printk("%u: ", net_pkt_get_len(pkt));
	for (int i = 0; i < net_pkt_get_len(pkt); i++) {
		printk("%c", data[i]);
	}
	printk("\n");
	return NET_OK;
}
