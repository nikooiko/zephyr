/*
 * Copyright (c) 2019 Nikos Oikonomou <nikoikonomou92@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define LOG_MODULE_NAME s2lp
#define LOG_LEVEL CONFIG_IEEE802154_DRIVER_LOG_LEVEL

#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <errno.h>
#include <spi.h>
#include <gpio.h>
#include <net/ieee802154_radio.h>
#include <net/net_core.h>
#include <net/net_pkt.h>
#include <net/net_if.h>
#include <random/rand32.h>

#include "ieee802154_s2lp.h"
#include <MCU_Interface.h>
#include <S2LP_Radio.h>
#include <S2LP_Gpio.h>
#include <S2LP_Types.h>
#include <S2LP_Csma.h>
#include <S2LP_Qi.h>
#include <S2LP_Commands.h>
#include <S2LP_Timer.h>
#include <S2LP_Timer_ex.h>
#include <S2LP_PacketHandler.h>
#include <S2LP_PktBasic.h>

/* Registers and Helpers */
#define HEADER_WRITE_MASK 0x00
#define HEADER_READ_MASK 0x01
#define HEADER_ADDRESS_MASK 0x00
#define HEADER_COMMAND_MASK 0x80

#define BUILT_HEADER(add_comm, w_r) (add_comm | w_r)
enum header_byte_t {
	WRITE_HEADER = BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_WRITE_MASK),
	READ_HEADER = BUILT_HEADER(HEADER_ADDRESS_MASK, HEADER_READ_MASK),
	COMMAND_HEADER = BUILT_HEADER(HEADER_COMMAND_MASK, HEADER_WRITE_MASK),
};

#define LINEAR_FIFO_ADDRESS 0xFF

/* Radio Default Configuration */
#define XTAL_FREQUENCY 50e6 /* Hz */
#define BASE_FREQUENCY 868e6
#define DATARATE 38400
#define FREQ_DEVIATION 20e3
#define BANDWIDTH 100E3
#define POWER_INDEX 7
#define RSSI_FILTER_GAIN 14
#define RSSI_MODE RSSI_STATIC_MODE
#define RSSI_RX_THRESHOLD -120.0 /* dBm */
#define RSSI_TX_THRESHOLD -90.0 /* dBm */
#define SYNC_WORD 0x88888888
#define CRC_MODE PKT_CRC_MODE_16BITS_1
#define EN_ADDRESS S_DISABLE
#define EN_FEC S_DISABLE
#define EN_WHITENING S_ENABLE
#define CHANNEL_SPACE 100e3
#define CHANNEL_NUMBER 0
#define MODULATION_SELECT MOD_2FSK
#define POWER_DBM 12.0
#define PREAMBLE_LENGTH PREAMBLE_BYTE(4)
#define SYNC_LENGTH SYNC_BYTE(4)
#define VARIABLE_LENGTH S_ENABLE
#define EXTENDED_LENGTH_FIELD S_DISABLE

#define PREAMBLE_BYTE(v) (4 * v)
#define SYNC_BYTE(v) (8 * v)

#define MIN_PERIOD_WAKEUP_MS ((8000 * ((PREAMBLE_LENGTH / 4) - 2)) / DATARATE)
#define RX_TIMEOUT_MS 30

#define EN_AUTOACK S_DISABLE
#define EN_PIGGYBACKING S_DISABLE
#define MAX_RETRANSMISSIONS 0

/* CSMA Configuration */
#define PERSISTENT_MODE_EN S_DISABLE
#define CS_PERIOD CSMA_PERIOD_64TBIT
#define CS_TIMEOUT 3
#define MAX_NB 5
#define BU_COUNTER_SEED 0xFA21
#define CU_PRESCALER 32

/* Validators */
#define TX_FIFO_LENGTH 127
#define MTU TX_FIFO_LENGTH
/*
 * TODO: this cannot be used right now, must uncomment later on
 *  #if (EXTENDED_LENGTH_FIELD == S_DISABLE)
 *      #define MTU                   TX_FIFO_LENGTH
 *  #else
 *      #define MTU                   1280
 *  #endif
 */

#define MAX_PA_VALUE 14
#define MIN_PA_VALUE -31
#define IS_PAPOWER_DBM(PATABLE)                                                \
	((PATABLE) >= (MIN_PA_VALUE) && (PATABLE) <= (MAX_PA_VALUE))

/**
 * @brief Generates a new mac address to be used by the network iface.
 * @param dev.
 * @retval The MAC address as 8 byte array.
 */
static inline u8_t *get_mac(struct device *dev)
{
	struct s2lp_802154_data *drv_data = dev->driver_data;
	u32_t *ptr = (u32_t *)(drv_data->mac);

	UNALIGNED_PUT(sys_rand32_get(), ptr);
	ptr = (u32_t *)(drv_data->mac + 4);
	UNALIGNED_PUT(sys_rand32_get(), ptr);

	/*
	 * Clear bit 0 to ensure it isn't a multicast address and set
	 * bit 1 to indicate address is locally administrered and may
	 * not be globally unique.
	 */
	drv_data->mac[0] = (drv_data->mac[0] & ~0x01) | 0x02;

	return drv_data->mac;
}

/**
 * @brief Helper method for decoding status from byte array format.
 * @param buf.
 * @retval None.
 */
static S2LPStatus decode_status(u8_t *buf)
{
	((u8_t *)&g_xStatus)[1] = buf[0];
	((u8_t *)&g_xStatus)[0] = buf[1];
	return g_xStatus;
}

/**
 * @brief Waits until the radio switches to requested state.
 * @param state.
 * @retval None.
 *
 * @note The calling thread will busy wait until radio switches to
 *      the requested state.
 */
static void wait_until_state(S2LPState state)
{
	while (g_xStatus.MC_STATE != state) {
		S2LPRefreshStatus();
	}
}

/**
 * @brief Commands radio to enter ready mode.
 * @param drv_data.
 * @retval None.
 *
 * @note This is the default state that the radio must be
 *      before switching to other modes, as instructed by the
 *      radio's datasheet. At this mode most functionality is
 *      disabled and ready to be enabled by another mode.
 */
static void enter_ready(struct s2lp_802154_data *drv_data)
{
	/* Make sure LDC is disabled to avoid errors during ready */
	S2LPTimerLdcrMode(S_DISABLE);
	S2LpTimerFastRxTermTimer(S_DISABLE);

	/* Enter Ready */
	/* If no operation for a while, do S2LPRefreshStatus() */
	if (g_xStatus.MC_STATE != MC_STATE_READY) {
		S2LPCmdStrobeSabort();
		S2LPCmdStrobeReady();
		wait_until_state(MC_STATE_READY);
	}
	gpio_pin_disable_callback(drv_data->rx_rdy_irq_gpio,
				  DT_ST_S2LP_0_RX_RDY_IRQ_GPIOS_PIN);
}

/**
 * @brief Commands radio to enter standby mode.
 * @param drv_data.
 * @retval None.
 *
 * @note To enter standby mode the radio must first switch to ready
 *      mode, as instructed at the radio's datasheet.
 */
static void enter_standby(struct s2lp_802154_data *drv_data)
{
	S2LPRefreshStatus();
	if (g_xStatus.MC_STATE != MC_STATE_READY) {
		enter_ready(drv_data);
	}

	/* Make timers are disabled to optimize power consumption */
	S2LPTimerLdcrMode(S_DISABLE);
	S2LpTimerFastRxTermTimer(S_DISABLE);

	/* Enter Standby */
	S2LPCmdStrobeStandby();
	wait_until_state(MC_STATE_STANDBY);
}

/**
 * @brief Commands radio to enter rx mode.
 * @param drv_data.
 * @retval None.
 *
 * @note To enter rx mode the radio must first switch to ready
 *      mode, as instructed at the radio's datasheet.
 */
static void enter_rx(struct s2lp_802154_data *drv_data)
{
	S2LPRefreshStatus();
	if (g_xStatus.MC_STATE != MC_STATE_READY) {
		enter_ready(drv_data);
	}

	/* Make sure LDC is enabled to optimize Rx */
	S2LPTimerLdcrMode(S_ENABLE);
	S2LpTimerFastRxTermTimer(S_ENABLE);
	S2LPCmdStrobeSleep();

	/* Enter Rx */
	S2LPCmdStrobeRx();
	wait_until_state(MC_STATE_RX);
	gpio_pin_enable_callback(drv_data->rx_rdy_irq_gpio,
				 DT_ST_S2LP_0_RX_RDY_IRQ_GPIOS_PIN);
}

/**
 * @brief Retrieves and returns the radio LQI value.
 * @param None.
 * @retval The output LQI value.
 */
static u8_t get_lqi(void)
{
	u8_t pqi;

	g_xStatus = RadioSpiReadRegisters(LINK_QUALIF2_ADDR, 1, &pqi);
	/* reduce to only 4 MSBit */
	return (pqi >> 4);
}

/**
 * @brief Retrieves the number of tx elements stored in FIFO.
 * @param None.
 * @retval The number of remaining tx elements
 */
static u8_t tx_remaining(void)
{
	return S2LPFifoReadNumberBytesTxFifo() & NELEM_TXFIFO_REGMASK;
}

/**
 * @brief Does an SPI operation.
 * @param drv_data
 * @param header WRITE_HEADER|READ_HEADER|COMMAND_HEADER
 * @param addr The target SPI register
 * @param data Byte array containing the incoming or outgoing data
 * @param n Size of data array
 * @param status_buf The radio status in byte array format
 * @retval 0 If successful, negative error code otherwise.
 *
 * @note To enter rx mode the radio must first switch to ready
 *      mode, as instructed at the radio datasheet.
 */
static int spi_op(struct s2lp_802154_data *drv_data, enum header_byte_t header,
		  u8_t addr, u8_t *data, u8_t n, u8_t *status_buf)
{
	int status;

	/* Prepare outgoing data buffers */
	u8_t out_buf[2] = { header, addr };
	struct spi_buf tx_spi_buf[2] = { {
						 .buf = out_buf,
						 .len = 2,
					 },
					 {
						 .buf = data,
						 .len = n,
					 } };
	struct spi_buf_set tx = {
		.buffers = tx_spi_buf,
		.count = header == WRITE_HEADER ?
				 2 :
				 1 /* data out only when write */
	};

	/* Prepare incoming data buffers */
	struct spi_buf rx_spi_buf[2] = {
		{
			.buf = status_buf,
			.len = 2,
		},
		{
			.buf = data,
			.len = n,
		},
	};
	struct spi_buf_set rx = {
		.buffers = rx_spi_buf,
		.count = header == READ_HEADER ? 2 :
						 1 /* data in only when read */
	};

	status = spi_transceive(drv_data->spi, &drv_data->spi_cfg, &tx, &rx);
	if (status < 0) {
		return status;
	}

	return status;
}

static inline void rx_rdy_handler(struct device *port, struct gpio_callback *cb,
				  u32_t pins)
{
	struct s2lp_802154_data *drv_data =
		CONTAINER_OF(cb, struct s2lp_802154_data, rx_rdy_cb);

	k_sem_give(&drv_data->isr_sem);
}

static void handle_rx_packet(struct s2lp_802154_data *drv_data, u8_t *payload,
			     u16_t length)
{
	struct net_pkt *pkt = NULL;

	/* Create packet and fill with data */
	pkt = net_pkt_alloc_with_buffer(drv_data->iface, length, AF_UNSPEC, 0,
					K_NO_WAIT);
	if (!pkt) {
		LOG_ERR("No pkt available");
		goto rx_packet_clean;
	}

	memcpy(net_pkt_data(pkt), payload, length);
	net_buf_add(pkt->buffer, length);
	net_pkt_set_ieee802154_lqi(pkt, get_lqi());
	net_pkt_set_ieee802154_rssi(pkt, S2LPRadioGetRssidBm());

	/* Check if pkt is ack to handle */
	if (ieee802154_radio_handle_ack(drv_data->iface, pkt) == NET_OK) {
		LOG_DBG("ACK handled");
		goto rx_packet_clean;
	}

	/* Forward to L3 */
	LOG_DBG("Pkt len=%u, rssi=%d, lqi=%u", length,
		(s8_t)net_pkt_ieee802154_rssi(pkt),
		net_pkt_ieee802154_lqi(pkt));
	if (net_recv_data(drv_data->iface, pkt) < 0) {
		LOG_DBG("Pkt dropped");
		goto rx_packet_clean;
	}

	net_analyze_stack("S2LP Rx stack",
			  Z_THREAD_STACK_BUFFER(drv_data->rx_stack),
			  K_THREAD_STACK_SIZEOF(drv_data->rx_stack));
rx_packet_clean:
	if (pkt) {
		net_pkt_unref(pkt);
		pkt = NULL;
	}
}

int retrieve_packet(struct s2lp_802154_data *drv_data, u8_t *pkt,
		    u16_t *pkt_len)
{
	S2LPIrqs x_irq_status;
	u16_t idx = 0;
	u8_t rx_n = S2LPFifoReadNumberBytesRxFifo();
	*pkt_len = 0;

	while (rx_n) {
		S2LPGpioIrqGetStatus(&x_irq_status);
		/* start with some validations to be safe */
		if (x_irq_status.IRQ_RX_FIFO_ERROR) {
			LOG_WRN("Rx FIFO error");
			return -1;
		} else if (x_irq_status.IRQ_CRC_ERROR) {
			LOG_WRN("Rx CRC error");
			return -1;
		} else if (x_irq_status.IRQ_RX_DATA_DISC) {
			LOG_WRN("Discarded rx data");
			return -1;
		} else if (idx >= MTU) {
			LOG_WRN("Not enough room for payload");
			return -1;
		}
		if (x_irq_status.IRQ_RSSI_ABOVE_TH) {
			/* TODO: define an action for this? */
			LOG_DBG("Rssi above threshold detected");
		}

		/* check for finished packet inside fifo */
		if (x_irq_status.IRQ_RX_DATA_READY) {
			*pkt_len = S2LPPktBasicGetReceivedPktLength();
			LOG_DBG("Rx data ready: len=%u, idx=%u",
				*pkt_len, idx);
			RadioSpiReadFifo(*pkt_len - idx, &pkt[idx]);
			break;
		}

		/* retrieve next byte until packet length is reported */
		RadioSpiReadFifo(1, &pkt[idx++]);

		/* continue processing */
		rx_n = S2LPFifoReadNumberBytesRxFifo();
		/*
		 * If no bytes in queue then wait to catch
		 * the upcoming payload.
		 * This is needed because in some cases, usually
		 * with large payloads, the rx processing is faster
		 * than the rx fifo fill, so must wait to be added
		 * in the rx fifo.
		 *
		 * TODO: maybe change to loop with retries?
		 */
		if (!rx_n) {
			LOG_DBG("Waiting for upcoming payload");
			/* TODO: define this based on Datarate? */
			k_sleep(1);
			rx_n = S2LPFifoReadNumberBytesRxFifo();
			if (!rx_n) {
				/* clear everything to avoid errors */
				return -1;
			}
		}
	}
	return 0;
}

static void check_rx_fifo(struct s2lp_802154_data *drv_data)
{
	u8_t pkt[MTU];
	u16_t pkt_len;

	do {
		if (retrieve_packet(drv_data, pkt, &pkt_len) < 0) {
			S2LPCmdStrobeFlushRxFifo();
			/* make sure to enter rx again after an error */
			enter_rx(drv_data);
			break;
		}

		if (pkt_len) {
			LOG_DBG("Pkt success len=%u", pkt_len);
			handle_rx_packet(drv_data, pkt, pkt_len);
		} else {
			LOG_DBG("No pkt found");
		}
	} while (S2LPFifoReadNumberBytesRxFifo());
}

static void s2lp_rx(void *p1, void *p2, void *p3)
{
	struct device *dev = (struct device *)p1;
	struct s2lp_802154_data *drv_data = dev->driver_data;

	while (true) {
		/* Wait until rx ready */
		LOG_DBG("Waiting for frame");
		k_sem_take(&drv_data->isr_sem, K_FOREVER);

		k_mutex_lock(&drv_data->phy_mutex, K_FOREVER);

		check_rx_fifo(drv_data);

		k_mutex_unlock(&drv_data->phy_mutex);
	}
}

/**
 * @brief Applies radio configuration
 * @param drv_data
 * @retval None.
 */
static void s2lp_radio_config(struct s2lp_802154_data *drv_data)
{
	S2LPRadioInit(&drv_data->x_radio_init);
	S2LPRadioSetChannel(CHANNEL_NUMBER);
	S2LPRadioSetChannelSpace(CHANNEL_SPACE);

	/* Tx power */
	S2LPRadioSetAutoRampingMode(S_DISABLE);
	S2LPRadioSetMaxPALevel(S_DISABLE);
	S2LPRadioSetPALeveldBm(POWER_INDEX, POWER_DBM);
	S2LPRadioSetPALevelMaxIndex(POWER_INDEX);

	/* Set Packet format */
	S2LPPktBasicInit(&drv_data->x_basic_init);

	/* CSMA config */
	S2LPCsmaInit(&drv_data->x_csma_init);
	S2LPCsma(S_ENABLE);
	S2LPRadioRssiInit(&drv_data->x_rssi_init);
	S2LPRadioSetPqiCheck(S_DISABLE);
	S2LPRadioSetPqiCheck(S_ENABLE);
	S2LPRadioSetRssiThreshdBm(RSSI_RX_THRESHOLD);

	/* Enable the following interrupt sources, routed to GPIO */
	S2LPGpioIrqDeInit(NULL);
	S2LPGpioIrqClearStatus();
	S2LPGpioIrqConfig(TX_DATA_SENT, S_ENABLE);
	S2LPGpioIrqConfig(RX_DATA_READY, S_ENABLE);
	/* Sniff mode requires valid sync, rx disc and rx timeout disabled */
	S2LPGpioIrqConfig(VALID_SYNC, S_DISABLE);
	S2LPGpioIrqConfig(RX_DATA_DISC, S_DISABLE);
	S2LPGpioIrqConfig(RX_TIMEOUT, S_DISABLE);
	S2LPGpioIrqConfig(RX_FIFO_ALMOST_FULL, S_ENABLE);
	/* CSMA requires max cca reach enabled */
	S2LPGpioIrqConfig(MAX_BO_CCA_REACH, S_ENABLE);
	S2LPGpioIrqConfig(TX_FIFO_ERROR, S_ENABLE);
	S2LPGpioIrqConfig(RX_FIFO_ERROR, S_ENABLE);

	/* Sniff Mode config */
	S2LPTimerSetWakeUpTimerMs(MIN_PERIOD_WAKEUP_MS);
	S2LPTimerSetRxTimerMs(RX_TIMEOUT_MS);
	S2LPTimerSleepB(S_ENABLE); /* +CSMA requires SLEEP_B mode */
	S2LPTimerLdcrMode(S_ENABLE);
	S2LpTimerFastRxTermTimer(S_ENABLE);
	S2LPTimerSetRxTimerCounter(0);
	S2LPPacketHandlerSetRxPersistentMode(S_ENABLE);

	/* Configure RX Ready IRQ */
	S2LPGpioInit(&drv_data->x_gpio_rx_rdy_irq);

	/* Finalize configuration */
	S2LPCsma(S_DISABLE);
}

/* Define Radio Device */

static int s2lp_init(struct device *dev)
{
	struct s2lp_802154_data *drv_data = dev->driver_data;

	/* Initialize Synchronization tools */
	k_mutex_init(&drv_data->phy_mutex);
	k_sem_init(&drv_data->isr_sem, 0, 1);

	/* Initialize Radio SPI Interface */
	RadioSpiInit();
	if (!drv_data->spi || !drv_data->cs_ctrl.gpio_dev) {
		return -ENODEV;
	}

	/* Configure Shutdown GPIO */
	drv_data->sdn_gpio =
		device_get_binding(DT_ST_S2LP_0_SDN_GPIOS_CONTROLLER);
	if (!drv_data->sdn_gpio) {
		LOG_ERR("Unable to get SDN GPIO device");
		return -ENODEV;
	}
	gpio_pin_configure(drv_data->sdn_gpio, DT_ST_S2LP_0_SDN_GPIOS_PIN,
			   DT_ST_S2LP_0_SDN_GPIOS_FLAGS);

	/* This must be set before all other settings */
	S2LPRadioSetXtalFrequency(XTAL_FREQUENCY);

	/* Reset Radio */
	RadioEnterShutdown();
	RadioExitShutdown(); /* sleeps for around 1 ms */

	/* wait at least 1.5 ms to allow Radio a proper boot-up sequence */
	k_sleep(1); /* already waited some more after shutdown exit */
	LOG_DBG("Radio Boot");

	/* Soft reset of core */
	S2LPCmdStrobeCommand(CMD_SRES);

	s2lp_radio_config(drv_data);

	/* Configure RX Ready IRQ GPIO */
	drv_data->rx_rdy_irq_gpio =
		device_get_binding(DT_ST_S2LP_0_RX_RDY_IRQ_GPIOS_CONTROLLER);
	if (!drv_data->rx_rdy_irq_gpio) {
		LOG_ERR("Unable to get RX Ready IRQ GPIO device");
		return -ENODEV;
	}
	gpio_pin_configure(drv_data->rx_rdy_irq_gpio,
			   DT_ST_S2LP_0_RX_RDY_IRQ_GPIOS_PIN,
			   DT_ST_S2LP_0_RX_RDY_IRQ_GPIOS_FLAGS);
	gpio_init_callback(&drv_data->rx_rdy_cb, rx_rdy_handler,
			   BIT(DT_ST_S2LP_0_RX_RDY_IRQ_GPIOS_PIN));
	gpio_add_callback(drv_data->rx_rdy_irq_gpio, &drv_data->rx_rdy_cb);

	/* Start Rx Thread */
	k_thread_create(&drv_data->rx_thread, drv_data->rx_stack,
			CONFIG_IEEE802154_S2LP_RX_STACK_SIZE,
			(k_thread_entry_t)s2lp_rx, dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_IEEE802154_S2LP_RX_THREAD_PRIO), 0,
			K_NO_WAIT);
	k_thread_name_set(&drv_data->rx_thread, "S2LP RX");

	/* Enter lpm until radio operation is started */
	enter_standby(drv_data);

	LOG_INF("Driver initialized successfully");
	return 0;
}

static void s2lp_iface_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct s2lp_802154_data *drv_data = dev->driver_data;

	get_mac(dev);
	net_if_set_link_addr(iface, drv_data->mac, sizeof(drv_data->mac),
			     NET_LINK_IEEE802154);

	drv_data->iface = iface;

	ieee802154_init(iface);
}

static enum ieee802154_hw_caps s2lp_get_capabilities(struct device *dev)
{
	/* TODO: can filter short addresses, enable IEEE802154_HW_FILTER? */
	/* TODO: can provide automatic acks, enable IEEE802154_HW_TX_RX_ACK? */
	return IEEE802154_HW_SUB_GHZ | IEEE802154_HW_CSMA | IEEE802154_HW_FCS;
}

static int s2lp_cca(struct device *dev)
{
	/* Because radio provides IEEE802154_HW_CSMA, function is not needed */
	LOG_WRN("Radio cca is not supported");
	return -ENOTSUP;
}

static int s2lp_set_channel(struct device *dev, u16_t channel)
{
	LOG_WRN("Radio set channel not implemented");
	return 0;
}

static u16_t s2lp_get_channel_count(struct device *dev)
{
	LOG_WRN("Radio get channel count not implemented");
	return 0xFFFF;
}

static int s2lp_filter(struct device *dev, bool set,
		       enum ieee802154_filter_type type,
		       const struct ieee802154_filter *filter)
{
	/* Radio doesn't provide IEEE802154_HW_FILTER, function is not needed */
	LOG_WRN("Radio filter not implemented");
	return -ENOTSUP;
}

static int s2lp_set_txpower(struct device *dev, s16_t dbm)
{
	struct s2lp_802154_data *drv_data = dev->driver_data;

	if (!IS_PAPOWER_DBM(dbm)) {
		LOG_ERR("Failure (dbm=%d)", dbm);
		return -EIO;
	}

	k_mutex_lock(&drv_data->phy_mutex, K_FOREVER);
	S2LPRadioSetPALeveldBm(POWER_INDEX, dbm);
	k_mutex_unlock(&drv_data->phy_mutex);

	LOG_INF("Radio set tx power (dbm=%d)", dbm);
	return 0;
}

static int s2lp_start(struct device *dev)
{
	struct s2lp_802154_data *drv_data = dev->driver_data;

	k_mutex_lock(&drv_data->phy_mutex, K_FOREVER);
	enter_rx(dev->driver_data);
	k_mutex_unlock(&drv_data->phy_mutex);

	LOG_INF("Started");
	return 0;
}

static int s2lp_stop(struct device *dev)
{
	struct s2lp_802154_data *drv_data = dev->driver_data;

	k_mutex_lock(&drv_data->phy_mutex, K_FOREVER);
	enter_standby(drv_data);
	k_mutex_unlock(&drv_data->phy_mutex);

	LOG_INF("Stopped");
	return 0;
}

static int s2lp_tx(struct device *dev, struct net_pkt *pkt,
		   struct net_buf *frag)
{
	struct s2lp_802154_data *drv_data = dev->driver_data;
	int status = 0;
	S2LPIrqs x_irq_status;
	u8_t tx_n;

	if (frag->len > MTU) {
		LOG_ERR("Packet too big");
		return -EINVAL;
	}

	k_mutex_lock(&drv_data->phy_mutex, K_FOREVER);
	LOG_DBG("Sending...");

	/* Prepare for Tx */
	S2LPCsma(S_ENABLE);
	S2LPRadioSetRssiThreshdBm(RSSI_TX_THRESHOLD);
	enter_ready(drv_data);
	S2LPCmdStrobeFlushTxFifo();
	LOG_DBG("Prepared");

	/* Add data to tx fifo */
	S2LPPktBasicSetPayloadLength(frag->len);
	RadioSpiWriteFifo((uint8_t)frag->len, frag->data);
	tx_n = tx_remaining();
	if (tx_n != frag->len) {
		status = -EIO;
		LOG_ERR("Unable to fill Tx FIFO (tx_n=%u)", tx_n);
		goto tx_done;
	}
	LOG_DBG("Data are forwarded");

	/* Transmit */
	S2LPGpioIrqClearStatus();
	S2LPCmdStrobeTx();

	/* Wait until transmission finishes */
	do {
		k_sleep(1); /* this is needed for Tx to finish */
		S2LPGpioIrqGetStatus(&x_irq_status);
		S2LPGpioIrqClearStatus();
	} while (!x_irq_status.IRQ_TX_DATA_SENT || tx_remaining() != 0);

	/* Check for Tx failure */
	tx_n = tx_remaining();
	if (tx_n != 0 || !x_irq_status.IRQ_TX_DATA_SENT) {
		status = -EIO;
		LOG_ERR("Failure (tx_n=%u)", tx_n);
		goto tx_done;
	}

tx_done:
	/* Restart Rx */
	S2LPCsma(S_DISABLE);
	S2LPRadioSetRssiThreshdBm(RSSI_RX_THRESHOLD);
	enter_rx(drv_data);

	LOG_DBG("Sent");
	k_mutex_unlock(&drv_data->phy_mutex);
	return status;
}

static struct ieee802154_radio_api s2lp_radio_api = {
	.iface_api.init = s2lp_iface_init,

	.get_capabilities = s2lp_get_capabilities,
	.cca = s2lp_cca,
	.set_channel = s2lp_set_channel,
	.get_subg_channel_count = s2lp_get_channel_count,
	.filter = s2lp_filter,
	.set_txpower = s2lp_set_txpower,
	.start = s2lp_start,
	.stop = s2lp_stop,
	.tx = s2lp_tx,
};

static struct s2lp_802154_data s2lp_data = {
	.x_radio_init = { .lFrequencyBase = BASE_FREQUENCY,
			  .xModulationSelect = MODULATION_SELECT,
			  .lDatarate = DATARATE,
			  .lFreqDev = FREQ_DEVIATION,
			  .lBandwidth = BANDWIDTH },
	.x_basic_init = { .xPreambleLength = PREAMBLE_LENGTH,
			  .xSyncLength = SYNC_LENGTH,
			  .lSyncWords = SYNC_WORD,
			  .xFixVarLength = VARIABLE_LENGTH,
			  .cExtendedPktLenField = EXTENDED_LENGTH_FIELD,
			  .xCrcMode = CRC_MODE,
			  .xAddressField = EN_ADDRESS,
			  .xFec = EN_FEC,
			  .xDataWhitening = EN_WHITENING },
	.x_csma_init = {
	/*
	 * CCA may optionally be persistent, i.e., rather than
	 * entering backoff when the channel is found busy, CCA continues
	 * until the channel becomes idle or until the MCU stops it.
	 * The thinking behind using this option is to give the MCU the
	 * possibility of managing the CCA by itself, for instance, with the
	 * allocation of a transmission timer: this timer would start when MCU
	 * finishes sending out data to be transmitted, and would end when MCU
	 * expects that its transmission takes place, which would occur after a
	 * period of CCA.
	 * The choice of making CCA persistent should come from trading off
	 * transmission latency,under the direct control of the MCU, and power
	 * consumption, which would be greater due to a busy wait in reception
	 * mode.
	 */
		.xCsmaPersistentMode = PERSISTENT_MODE_EN,
		.xMultiplierTbit = CS_PERIOD,
		.xCcaLength = CS_TIMEOUT,
		.cMaxNb = MAX_NB,
		.nBuCounterSeed = BU_COUNTER_SEED,
		.cBuPrescaler = CU_PRESCALER
	},
	.x_rssi_init = {
		.cRssiFlt = RSSI_FILTER_GAIN,
		.xRssiMode = RSSI_MODE,
		.cRssiThreshdBm = RSSI_TX_THRESHOLD /* dBm */
	},
	.x_gpio_rx_rdy_irq = { .xS2LPGpioPin = S2LP_GPIO_3,
			       .xS2LPGpioMode =
				       S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP,
			       .xS2LPGpioIO = S2LP_GPIO_DIG_OUT_IRQ }
};

#if defined(CONFIG_IEEE802154_RAW_MODE)
DEVICE_AND_API_INIT(s2lp, CONFIG_IEEE802154_S2LP_DRV_NAME, s2lp_init,
		    &s2lp_data, NULL, POST_KERNEL,
		    CONFIG_IEEE802154_S2LP_INIT_PRIO, &s2lp_radio_api);
#else
NET_DEVICE_INIT(s2lp, CONFIG_IEEE802154_S2LP_DRV_NAME, s2lp_init, &s2lp_data,
		NULL, CONFIG_IEEE802154_S2LP_INIT_PRIO, &s2lp_radio_api,
		IEEE802154_L2, NET_L2_GET_CTX_TYPE(IEEE802154_L2), MTU);

NET_STACK_INFO_ADDR(RX, s2lp_radio, CONFIG_IEEE802154_S2LP_RX_STACK_SIZE,
		    CONFIG_IEEE802154_S2LP_RX_STACK_SIZE, s2lp_data.rx_stack,
		    0);
#endif

/* Implement MCU Interface for S2-LP Radio */
#include <S2LP_Types.h>

void RadioSpiInit(void)
{
	s2lp_data.spi = device_get_binding(DT_ST_S2LP_0_BUS_NAME);
	if (!s2lp_data.spi) {
		LOG_ERR("Unable to get SPI device");
		return;
	}

	s2lp_data.cs_ctrl.gpio_dev =
		device_get_binding(DT_ST_S2LP_0_CS_GPIO_CONTROLLER);
	if (!s2lp_data.cs_ctrl.gpio_dev) {
		LOG_ERR("Unable to get SPI CS GPIO device");
		return;
	}
	s2lp_data.cs_ctrl.gpio_pin = DT_ST_S2LP_0_CS_GPIO_PIN;
	s2lp_data.cs_ctrl.delay = 0U;

	s2lp_data.spi_cfg.frequency = DT_ST_S2LP_0_SPI_MAX_FREQUENCY;
	s2lp_data.spi_cfg.operation = SPI_WORD_SET(8);
	s2lp_data.spi_cfg.slave = DT_ST_S2LP_0_BASE_ADDRESS;
	s2lp_data.spi_cfg.cs = &s2lp_data.cs_ctrl;

	LOG_DBG("S2LP SPI initialized");
}

/**
 * @brief Write single or multiple registers.
 * @param cRegAddress Base register's address to be write
 * @param cNbBytes Number of registers and bytes to be write
 * @param pcBuffer Pointer to the buffer of values have to be written into
 * registers
 * @retval The radio status
 */
StatusBytes RadioSpiWriteRegisters(uint8_t cRegAddress, uint8_t cNbBytes,
				   uint8_t *pcBuffer)
{
	int status;
	u8_t status_buf[2] = { 0 };

	status = spi_op(&s2lp_data, WRITE_HEADER, cRegAddress, pcBuffer,
			cNbBytes, status_buf);
	if (status < 0) {
		LOG_ERR("Failed to write registers (err=%d, addr=%u)", status,
			cRegAddress);
	}

	return decode_status(status_buf);
}

/**
 * @brief Read single or multiple registers.
 * @param cRegAddress Base register's address to be read
 * @param cNbBytes Number of registers and bytes to be read
 * @param pcBuffer Pointer to the buffer of registers' values read
 * @retval The radio status
 */
StatusBytes RadioSpiReadRegisters(uint8_t cRegAddress, uint8_t cNbBytes,
				  uint8_t *pcBuffer)
{
	int status;
	u8_t status_buf[2] = { 0 };

	status = spi_op(&s2lp_data, READ_HEADER, cRegAddress, pcBuffer,
			cNbBytes, status_buf);
	if (status < 0) {
		LOG_ERR("Failed to read registers (err=%d, addr=%u)", status,
			cRegAddress);
	}

	return decode_status(status_buf);
}

/**
 * @brief Send a command
 * @param cCommandCode Command code to be sent
 * @retval The radio status
 */
StatusBytes RadioSpiCommandStrobes(uint8_t cCommandCode)
{
	int status;
	u8_t status_buf[2] = { 0 };

	status = spi_op(&s2lp_data, COMMAND_HEADER, cCommandCode, NULL, 0,
			status_buf);
	if (status < 0) {
		LOG_ERR("Failed to write registers (err=%d, code=%u)", status,
			cCommandCode);
	}

	return decode_status(status_buf);
}

/**
 * @brief Write data into TX FIFO.
 * @param cNbBytes Number of bytes to be written into TX FIFO
 * @param pcBuffer Pointer to data to write
 * @retval The radio status
 */
StatusBytes RadioSpiWriteFifo(uint8_t cNbBytes, uint8_t *pcBuffer)
{
	return RadioSpiWriteRegisters(LINEAR_FIFO_ADDRESS, cNbBytes, pcBuffer);
}

/**
 * @brief Read data from RX FIFO.
 * @param cNbBytes Number of bytes to read from RX FIFO
 * @param pcBuffer Pointer to data read from RX FIFO
 * @retval The radio status
 */
StatusBytes RadioSpiReadFifo(uint8_t cNbBytes, uint8_t *pcBuffer)
{
	return RadioSpiReadRegisters(LINEAR_FIFO_ADDRESS, cNbBytes, pcBuffer);
}

/**
 * @brief Puts at logic 1 the SDN pin.
 * @param None.
 * @retval None.
 */
void RadioEnterShutdown(void)
{
	gpio_pin_write(s2lp_data.sdn_gpio, DT_ST_S2LP_0_SDN_GPIOS_PIN, 1);
}

/**
 * @brief Put at logic 0 the SDN pin.
 * @param None.
 * @retval None.
 */
void RadioExitShutdown(void)
{
	gpio_pin_write(s2lp_data.sdn_gpio, DT_ST_S2LP_0_SDN_GPIOS_PIN, 0);
	/* delay to allow the circuit POR. Need about 700 us */
	k_sleep(1);
}
