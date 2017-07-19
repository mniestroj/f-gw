#include <zephyr.h>
#include <misc/printk.h>

#include <device.h>
#include <spi.h>

#include "common.h"
#include "nrf24.h"
#include "msg.h"

#ifdef VERBOSE_DEBUG

#ifndef vprintk
#define vprintk printk
#endif

#else

#ifndef vprintk
#define vprintk(fmt, arg...) do { } while (0)
#endif

#endif

K_SEM_DEFINE(nrf_ready_sem, 0, UINT_MAX);
static volatile int nrf_ready = 0;

static void nrf_irq_callback(struct device *port,
			struct gpio_callback *cb, uint32_t pin)
{
	struct nrf24 *nrf = CONTAINER_OF(cb, struct nrf24, irq_cb);
	ARG_UNUSED(nrf);

	nrf_ready = 1;
	k_sem_give(&nrf_ready_sem);
}

static struct nrf24_config nrf_config = {
	.spi_port = "SPI_0",

	.cs_port = "GPIO_0",
	.cs_pin = 11,

	.ce_port = "GPIO_0",
	.ce_pin = 12,

	.irq_port = "GPIO_0",
	.irq_pin = 20,
	.irq_callback = nrf_irq_callback,

	.payload_size = 0,
};

static struct nrf24 nrf;

static void nrf24_local_init(struct nrf24 *nrf)
{
	uint8_t val;

	nrf24_init(nrf);

	/* RF Speed and power*/
	nrf24_set_byte(nrf, NRF24_RF_SETUP, NRF24_RF_DR_250K | NRF24_RF_PWR_M6);

	/* Address */
	nrf24_set_rx_address(nrf, 0, 0x0102030405);

	/* Set to RX mode */
	val = nrf24_get_byte(nrf, NRF24_CONFIG);
	nrf24_set_byte(nrf, NRF24_CONFIG, val | NRF24_PRIM_RX);
	nrf24_set_byte(nrf, NRF24_STATUS, NRF24_RX_DR);
}

static void send_event(uint32_t sensor_addr, uint32_t timestamp,
		uint32_t event_type, uint32_t event_data)
{
	struct event event = {
		.sensor_addr = sensor_addr,
		.timestamp = timestamp,
		.event_type = event_type,
		.event_data = event_data,
	};

	while (k_msgq_put(&event_queue, &event, K_NO_WAIT) != 0)
		k_msgq_purge(&event_queue);
}

static void dump_payload(int pipe, uint8_t *payload, unsigned int len)
{
	struct msg_header *header = (struct msg_header *) payload;
	uint8_t i;

	printk("%d: [%d %2u] ", pipe, header->ts, len);

#define CASE_MSG_TYPE(type)			\
	case MSG_ ## type:			\
		printk(#type " len = %u", len);	\
		break

	switch (header->type) {
	CASE_MSG_TYPE(PWR_ON);
	case MSG_IR_ONOFF: {
		uint32_t value = ((((uint32_t) payload[MSG_HEADER_LEN + 0]) << 0) |
				(((uint32_t) payload[MSG_HEADER_LEN + 1]) << 8) |
				(((uint32_t) payload[MSG_HEADER_LEN + 2]) << 16) |
				(((uint32_t) payload[MSG_HEADER_LEN + 3]) << 24));
		printk("IR_ONOFF %u", (unsigned int) value);
		send_event(2, k_uptime_get() - header->ts,
			header->type, value);
		break;
	}
	CASE_MSG_TYPE(IR_USR);
	case MSG_MAG_ONOFF: {
		uint32_t value = ((((uint32_t) payload[MSG_HEADER_LEN + 0]) << 0) |
				(((uint32_t) payload[MSG_HEADER_LEN + 1]) << 8) |
				(((uint32_t) payload[MSG_HEADER_LEN + 2]) << 16) |
				(((uint32_t) payload[MSG_HEADER_LEN + 3]) << 24));
		printk("MAG_ONOFF %u", (unsigned int) value);
		send_event(1, k_uptime_get() - header->ts,
			header->type, value);
		break;
	}
	case MSG_MAG_RAW: {
		uint32_t raw = ((((uint32_t) payload[MSG_HEADER_LEN + 0]) << 0) |
				(((uint32_t) payload[MSG_HEADER_LEN + 1]) << 8) |
				(((uint32_t) payload[MSG_HEADER_LEN + 2]) << 16) |
				(((uint32_t) payload[MSG_HEADER_LEN + 3]) << 24));
		printk("MAG_RAW	  %u", (unsigned int) raw);
		break;
	}
	case MSG_STRING:
		printk("STRING '%s'", &payload[MSG_HEADER_LEN]);
		break;
	default:
		printk("Unknown <0x%02x>", (unsigned int) header->type);
	}

	/* as bytes */
	vprintk(" [");
	for (i = 0; i < len; i++)
		vprintk(" %02x", (unsigned int) payload[i]);
	vprintk(" ]");

	printk("\n");
}

void process_payload(int pipe, uint8_t *payload, unsigned int len)
{
	static unsigned int buf[6][10] = {{0}};
	static unsigned int sum[6] = {0};
	static unsigned int idx[6] = {0};
	struct msg_header *header = (struct msg_header *) payload;
	unsigned int *pidx = &idx[pipe];
	unsigned int *psum = &sum[pipe];
	unsigned int *pbuf = &buf[pipe][*pidx];
	uint16_t mag;
	uint16_t vref;

	if (len < MSG_HEADER_LEN + 4 || header->type != MSG_MAG_RAW)
		return;

	mag = ((((uint16_t) payload[MSG_HEADER_LEN + 0]) << 0) |
		(((uint16_t) payload[MSG_HEADER_LEN + 1]) << 8));

	vref = ((((uint16_t) payload[MSG_HEADER_LEN + 2]) << 0) |
		(((uint16_t) payload[MSG_HEADER_LEN + 3]) << 8));

	*psum -= *pbuf;
	*pbuf = (1000000 * mag) / vref;
	*psum += *pbuf;

	if (*pidx == 0)
		printk("[%d] [%d] mag: %u, vref: %u, avg: %u\n",
			pipe, (int) header->ts,
			(unsigned int) mag,
			(unsigned int) vref,
			(unsigned int) (*psum / ARRAY_SIZE(buf[0])));
	*pidx = (*pidx + 1) % ARRAY_SIZE(buf[0]);
}

void nrf_main(void *p1, void *p2, void *p3)
{
	uint8_t rx_payload[32];
	uint8_t status;
	uint8_t fifo_status;
	int ret;

	printk("Hello World! %s\n", CONFIG_ARCH);
	printk("Initializing nrf24\n");

	/* nRF24 init */
	nrf.config = &nrf_config;
	nrf24_local_init(&nrf);

	/* power up */
	printk("Powering up\n");
	nrf24_power(&nrf, 1);

	/* Prepare receiver */
	printk("Enabling receiver\n");
	nrf24_ce(&nrf, 1);

	status = nrf24_status(&nrf);
	printk("STATUS = 0x%02x\n", (unsigned int) status);

	fifo_status = nrf24_get_byte(&nrf, NRF24_FIFO_STATUS);
	printk("FIFO_STATUS = 0x%02x\n", (unsigned int) fifo_status);

	while (1) {
		vprintk("Waiting for data\n");
		while (!nrf_ready)
			k_sem_take(&nrf_ready_sem, K_FOREVER);
		nrf_ready = 0;
		vprintk("There was an IRQ!\n");

		status = nrf24_status(&nrf);
		vprintk("STATUS = 0x%02x\n", (unsigned int) status);

		/* This is needed in case there was an interrupt (somehow) triggered but there is no data in RX FIFO */
		if (status & NRF24_RX_DR) {
			vprintk("Clearing NRF24_RX_DR\n");
			nrf24_set_byte(&nrf, NRF24_STATUS, NRF24_RX_DR);
			status &= ~(NRF24_RX_DR);
		} else {
			vprintk("Clearing NRF24_other interrupt\n");
			nrf24_set_byte(&nrf, NRF24_STATUS, status & (NRF24_TX_DS | NRF24_MAX_RT));
			status &= ~(NRF24_TX_DS | NRF24_MAX_RT);
		}

		while ((status & NRF24_RX_P_NO_MASK) != NRF24_RX_P_NO_MASK /*!(fifo_status & NRF24_FIFO_RX_EMPTY)*/) {
			if (status & NRF24_RX_DR) {
				vprintk("Clearing NRF24_RX_DR\n");
				nrf24_set_byte(&nrf, NRF24_STATUS, NRF24_RX_DR);
			}

			ret = nrf24_receive(&nrf, rx_payload);
			if (ret < 0) {
				vprintk("Received invalid packet (PL_WID invalid)\n");
				break;
			}
			dump_payload(NRF24_RX_P_NO(status), rx_payload, ret);

			status = nrf24_status(&nrf);
			vprintk("STATUS = 0x%02x\n", (unsigned int) status);
		}
	}
}
