#pragma once

#include <zephyr.h>
#include <gpio.h>

/* Commands */
#define NRF24_R_REGISTER	0x00
#define NRF24_W_REGISTER	0x20
#define NRF24_R_RX_PAYLOAD	0x61
#define NRF24_W_TX_PAYLOAD	0xA0
#define NRF24_FLUSH_TX		0xE1
#define NRF24_FLUSH_RX		0xE2
#define NRF24_REUSE_TX_PL	0xE3
#define NRF24_R_RX_PL_WID	0x60
#define NRF24_NOP		0xFF

/* Register addresses */
#define NRF24_CONFIG		0x00
#define NRF24_EN_AA		0x01
#define NRF24_EN_RXADDR		0x02
#define NRF24_SETUP_AW		0x03
#define NRF24_SETUP_RETR	0x04
#define NRF24_RF_CH		0x05
#define NRF24_RF_SETUP		0x06
#define NRF24_STATUS		0x07
#define NRF24_OBSERVE_TX	0x08
#define NRF24_RPD		0x09
#define NRF24_RX_ADDR_P0	0x0A
#define NRF24_RX_ADDR_P1	0x0B
#define NRF24_RX_ADDR_P2	0x0C
#define NRF24_RX_ADDR_P3	0x0D
#define NRF24_RX_ADDR_P4	0x0E
#define NRF24_RX_ADDR_P5	0x0F
#define NRF24_TX_ADDR		0x10
#define NRF24_RX_PW_P0		0x11
#define NRF24_RX_PW_P1		0x12
#define NRF24_RX_PW_P2		0x13
#define NRF24_RX_PW_P3		0x14
#define NRF24_RX_PW_P4		0x15
#define NRF24_RX_PW_P5		0x16
#define NRF24_FIFO_STATUS	0x17
#define NRF24_ACK_PLD		0x18
#define NRF24_DYN_PD		0x1C
#define NRF24_FEATURE		0x1D

/* CONFIG */
#define NRF24_MASK_RX_DR	BIT(6)
#define NRF24_MASK_TX_DR	BIT(5)
#define NRF24_MASK_MAX_RT	BIT(4)
#define NRF24_EN_CRC		BIT(3)
#define NRF24_CRCO		BIT(2)
#define NRF24_PWR_UP		BIT(1)
#define NRF24_PRIM_RX		BIT(0)

/* STATUS */
#define NRF24_RX_DR		BIT(6)
#define NRF24_TX_DS		BIT(5)
#define NRF24_MAX_RT		BIT(4)
#define NRF24_RX_P_NO_MASK	(0x7 << 1)
#define NRF24_RX_P_NO(val)	(((val) >> 1) & 0x7)
#define NRF24_TX_FULL		BIT(0)

/* RF_SETUP */
#define NRF24_RF_DR_250K	BIT(5)
#define NRF24_RF_DR_1M		0
#define NRF24_RF_DR_2M		BIT(3)
#define NRF24_RF_PWR_M18	0
#define NRF24_RF_PWR_M12	BIT(1)
#define NRF24_RF_PWR_M6		BIT(2)
#define NRF24_RF_PWR_0		(BIT(1) | BIT(2))

/* FIFO_STATUS */
#define NRF24_FIFO_TX_REUSE	BIT(6)
#define NRF24_FIFO_TX_FULL	BIT(5)
#define NRF24_FIFO_TX_EMPTY	BIT(4)
#define NRF24_FIFO_RX_FULL	BIT(1)
#define NRF24_FIFO_RX_EMPTY	BIT(0)

/* SETUP_RETR */
#define NRF24_ARD_250us(x)	((((x)-1) & 0xFF) << 4)
#define NRF24_ARC(x)		(x)

/* FEATURE */
#define NRF24_EN_DPL		BIT(2)
#define NRF24_EN_ACK_PAY	BIT(1)
#define NRF24_EN_DYN_ACK	BIT(0)

struct nrf24_config {
	char *spi_port;

	char*cs_port;
	uint8_t cs_pin;
	char *ce_port;
	uint8_t ce_pin;
	char *irq_port;
	uint8_t irq_pin;
	gpio_callback_handler_t irq_callback;

	uint8_t payload_size;
};

struct nrf24 {
	struct nrf24_config *config;
	struct device *spi_dev;
	struct device *cs;
	struct device *ce;
	struct device *irq;
	struct gpio_callback irq_cb;
	uint8_t status;
};

static inline void nrf24_ce(struct nrf24 *nrf, int enable)
{
	gpio_pin_write(nrf->ce, nrf->config->ce_pin, enable);
}

static inline void nrf24_cs(struct nrf24 *nrf, int enable)
{
	gpio_pin_write(nrf->cs, nrf->config->cs_pin, enable);
}

static inline int nrf24_irq(struct nrf24 *nrf)
{
	uint32_t val;
	gpio_pin_read(nrf->irq, nrf->config->irq_pin, &val);
	return !val;
}

uint8_t nrf24_get_byte(struct nrf24 *nrf, uint8_t reg);
void nrf24_set_byte(struct nrf24 *nrf, uint8_t reg, uint8_t val);

void nrf24_set_rx_address(struct nrf24 *nrf, uint8_t pipe, uint64_t address);
uint64_t nrf24_get_rx_address(struct nrf24 *nrf, uint8_t pipe);
void nrf24_set_tx_address(struct nrf24 *nrf, uint64_t address);
uint64_t nrf24_get_tx_address(struct nrf24 *nrf);

void nrf24_flush_tx(struct nrf24 *nrf);
void nrf24_flush_rx(struct nrf24 *nrf);

void nrf24_send(struct nrf24 *nrf, uint8_t *data, uint8_t size);
int nrf24_receive(struct nrf24 *nrf, uint8_t *data);

int nrf24_init(struct nrf24 *nrf);
void nrf24_power(struct nrf24 *nrf, int on);

uint8_t nrf24_status(struct nrf24 *nrf);
