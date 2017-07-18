#include "nrf24.h"

#include <logging/sys_log.h>
#include <spi.h>

static int nrf24_ce_init(struct nrf24 *nrf)
{
	const struct nrf24_config *cfg = nrf->config;

	nrf->ce = device_get_binding(cfg->ce_port);
	if (!nrf->ce) {
		SYS_LOG_DBG("GPIO controller %s not found.", cfg->ce_port);
		return -EINVAL;
	}

	gpio_pin_configure(nrf->ce, cfg->ce_pin, GPIO_DIR_OUT);

	return 0;
}

static int nrf24_cs_init(struct nrf24 *nrf)
{
	const struct nrf24_config *cfg = nrf->config;

	nrf->cs = device_get_binding(cfg->cs_port);
	if (!nrf->cs) {
		SYS_LOG_DBG("GPIO controller %s not found.", cfg->cs_port);
		return -EINVAL;
	}

	gpio_pin_configure(nrf->cs, cfg->cs_pin, GPIO_DIR_OUT);

	nrf24_cs(nrf, 1);

	return 0;
}

static int nrf24_irq_init(struct nrf24 *nrf)
{
	const struct nrf24_config *cfg = nrf->config;

	nrf->irq = device_get_binding(cfg->irq_port);
	if (!nrf->irq) {
		SYS_LOG_DBG("GPIO controller %s not found.", cfg->irq_port);
		return -EINVAL;
	}

	gpio_pin_configure(nrf->irq, cfg->irq_pin,
			GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
			GPIO_INT_ACTIVE_LOW | GPIO_INT_DEBOUNCE);

	gpio_init_callback(&nrf->irq_cb,
			cfg->irq_callback,
			BIT(cfg->irq_pin));

	gpio_add_callback(nrf->irq, &nrf->irq_cb);
	gpio_pin_enable_callback(nrf->irq, cfg->irq_pin);

	return 0;
}

uint8_t nrf24_get_byte(struct nrf24 *nrf, uint8_t reg)
{
	uint8_t data_out[] = { reg };
	uint8_t data_in[2];

	nrf24_cs(nrf, 0);
	spi_transceive(nrf->spi_dev, data_out, ARRAY_SIZE(data_out),
		       data_in, ARRAY_SIZE(data_in));
	nrf24_cs(nrf, 1);

	nrf->status = data_in[0];
	return data_in[1];
}

void nrf24_set_byte(struct nrf24 *nrf, uint8_t reg, uint8_t val)
{
	uint8_t data_out[] = { reg | NRF24_W_REGISTER, val };
	uint8_t data_in[1];

	nrf24_cs(nrf, 0);
	spi_transceive(nrf->spi_dev, data_out, ARRAY_SIZE(data_out),
		       data_in, ARRAY_SIZE(data_in));
	nrf24_cs(nrf, 1);

	nrf->status = data_in[0];
}

void nrf24_set_rx_address(struct nrf24 *nrf, uint8_t pipe, uint64_t address)
{
	uint8_t data_out[] = {
		(NRF24_RX_ADDR_P0 + pipe) | NRF24_W_REGISTER,
		(address >>  0) & 0xFF,
		(address >>  8) & 0xFF,
		(address >> 16) & 0xFF,
		(address >> 24) & 0xFF,
		(address >> 32) & 0xFF,
	};
	uint8_t data_in[1];

	nrf24_cs(nrf, 0);
	spi_transceive(nrf->spi_dev, data_out, ARRAY_SIZE(data_out),
		       data_in, ARRAY_SIZE(data_in));
	nrf24_cs(nrf, 1);

	nrf->status = data_in[0];
}

uint64_t nrf24_get_rx_address(struct nrf24 *nrf, uint8_t pipe)
{
	uint8_t data_out[] = { NRF24_RX_ADDR_P0 + pipe };
	uint8_t data_in[6];

	nrf24_cs(nrf, 0);
	spi_transceive(nrf->spi_dev, data_out, ARRAY_SIZE(data_out),
		       data_in, ARRAY_SIZE(data_in));
	nrf24_cs(nrf, 1);

	nrf->status = data_in[0];

	return ((((uint64_t) data_in[1]) <<  0) |
		(((uint64_t) data_in[2]) <<  8) |
		(((uint64_t) data_in[3]) << 16) |
		(((uint64_t) data_in[4]) << 24) |
		(((uint64_t) data_in[5]) << 32));
}

void nrf24_set_tx_address(struct nrf24 *nrf, uint64_t address)
{
	uint8_t data_out[] = {
		NRF24_TX_ADDR | NRF24_W_REGISTER,
		(address >>  0) & 0xFF,
		(address >>  8) & 0xFF,
		(address >> 16) & 0xFF,
		(address >> 24) & 0xFF,
		(address >> 32) & 0xFF,
	};
	uint8_t data_in[1];

	nrf24_cs(nrf, 0);
	spi_transceive(nrf->spi_dev, data_out, ARRAY_SIZE(data_out),
		       data_in, ARRAY_SIZE(data_in));
	nrf24_cs(nrf, 1);

	nrf->status = data_in[0];
}

uint64_t nrf24_get_tx_address(struct nrf24 *nrf)
{
	uint8_t data_out[] = { NRF24_TX_ADDR };
	uint8_t data_in[6];

	nrf24_cs(nrf, 0);
	spi_transceive(nrf->spi_dev, data_out, ARRAY_SIZE(data_out),
		       data_in, ARRAY_SIZE(data_in));
	nrf24_cs(nrf, 1);

	nrf->status = data_in[0];

	return ((((uint64_t) data_in[1]) <<  0) |
		(((uint64_t) data_in[2]) <<  8) |
		(((uint64_t) data_in[3]) << 16) |
		(((uint64_t) data_in[4]) << 24) |
		(((uint64_t) data_in[5]) << 32));
}

void nrf24_flush_tx(struct nrf24 *nrf)
{
	uint8_t data_out[] = { NRF24_FLUSH_TX };
	uint8_t data_in[1];

	nrf24_cs(nrf, 0);
	spi_transceive(nrf->spi_dev, data_out, ARRAY_SIZE(data_out),
		       data_in, ARRAY_SIZE(data_in));
	nrf24_cs(nrf, 1);

	nrf->status = data_in[0];
}

void nrf24_flush_rx(struct nrf24 *nrf)
{
	uint8_t data_out[] = { NRF24_FLUSH_RX };
	uint8_t data_in[1];

	nrf24_cs(nrf, 0);
	spi_transceive(nrf->spi_dev, data_out, ARRAY_SIZE(data_out),
		       data_in, ARRAY_SIZE(data_in));
	nrf24_cs(nrf, 1);

	nrf->status = data_in[0];
}

void nrf24_send(struct nrf24 *nrf, uint8_t *data, uint8_t size)
{
	uint8_t data_out[33] = { NRF24_W_TX_PAYLOAD };
	unsigned payload_size = nrf->config->payload_size;
	uint8_t status;
	uint8_t i;

	if (!payload_size)
		payload_size = size;

	for (i = 0; i < size; i++)
		data_out[i + 1] = data[i];

	nrf24_cs(nrf, 0);
	spi_transceive(nrf->spi_dev, data_out, payload_size + 1,
		       &status, 1);
	nrf24_cs(nrf, 1);

	nrf->status = status;
}

int nrf24_receive(struct nrf24 *nrf, uint8_t *data)
{
	uint8_t data_in[33];
	uint8_t command;
	unsigned int payload_size = nrf->config->payload_size;
	uint8_t i;

	if (!payload_size) {
		command = NRF24_R_RX_PL_WID;
		nrf24_cs(nrf, 0);
		spi_transceive(nrf->spi_dev, &command, 1,
			data_in, 2);
		nrf24_cs(nrf, 1);

		payload_size = data_in[1];

		if (payload_size > 32) {
			nrf24_flush_rx(nrf);
			return -1;
		}
	}

	command = NRF24_R_RX_PAYLOAD;
	nrf24_cs(nrf, 0);
	spi_transceive(nrf->spi_dev, &command, 1,
		       data_in, payload_size + 1);
	nrf24_cs(nrf, 1);

	for (i = 0; i < payload_size; i++)
		data[i] = data_in[i + 1];

	nrf->status = data_in[0];

	return payload_size;
}

static int nrf24_spi_init(struct nrf24 *nrf)
{
	struct spi_config config = {
		.config = SPI_WORD(8),
		.max_sys_freq = 1000000, /* 1 MHz */
	};

	nrf->spi_dev = device_get_binding(nrf->config->spi_port);
	if (!nrf->spi_dev)
		return -ENODEV;

	return spi_configure(nrf->spi_dev, &config);
}

int nrf24_init(struct nrf24 *nrf)
{
	int ret;
	uint8_t val;

	ret = nrf24_cs_init(nrf);
	if (ret < 0)
		return ret;

	ret = nrf24_ce_init(nrf);
	if (ret < 0)
		return ret;

	ret = nrf24_irq_init(nrf);
	if (ret < 0)
		return ret;

	ret = nrf24_spi_init(nrf);
	if (ret < 0)
		return ret;

	/* Disable radio */
	nrf24_ce(nrf, 0);

	/* Power down */
	nrf24_power(nrf, 0);

	/* Clear interrupt registers */
	val = nrf24_status(nrf);
	val &= (NRF24_RX_DR | NRF24_TX_DS | NRF24_MAX_RT);
	if (val)
		nrf24_set_byte(nrf, NRF24_STATUS, val);

	/* Clear FIFOs */
	val = nrf24_get_byte(nrf, NRF24_FIFO_STATUS);
	while ((val & (NRF24_FIFO_TX_EMPTY | NRF24_FIFO_RX_EMPTY)) !=
			(NRF24_FIFO_TX_EMPTY | NRF24_FIFO_RX_EMPTY)) {
		if (!(val & NRF24_FIFO_TX_EMPTY))
			nrf24_flush_tx(nrf);

		if (!(val & NRF24_FIFO_RX_EMPTY))
			nrf24_flush_rx(nrf);

		val = nrf24_get_byte(nrf, NRF24_FIFO_STATUS);
	}

	/* Set payload length */
	if (nrf->config->payload_size) { /* Static payload length */
		nrf24_set_byte(nrf, NRF24_RX_PW_P0, nrf->config->payload_size);
		nrf24_set_byte(nrf, NRF24_RX_PW_P1, nrf->config->payload_size);
		nrf24_set_byte(nrf, NRF24_RX_PW_P2, nrf->config->payload_size);
		nrf24_set_byte(nrf, NRF24_RX_PW_P3, nrf->config->payload_size);
		nrf24_set_byte(nrf, NRF24_RX_PW_P4, nrf->config->payload_size);
		nrf24_set_byte(nrf, NRF24_RX_PW_P5, nrf->config->payload_size);
	} else { /* Dynamic payload length */
		nrf24_set_byte(nrf, NRF24_FEATURE, NRF24_EN_DPL);
		nrf24_set_byte(nrf, NRF24_DYN_PD, 0x3F);
	}

	return ret;
}

void nrf24_power(struct nrf24 *nrf, int on)
{
	uint8_t val;

	val = nrf24_get_byte(nrf, NRF24_CONFIG);
	if (on)
		val |= NRF24_PWR_UP;
	else
		val &= ~NRF24_PWR_UP;

	nrf24_set_byte(nrf, NRF24_CONFIG, val);
}

uint8_t nrf24_status(struct nrf24 *nrf)
{
	uint8_t data_out[] = { NRF24_NOP };
	uint8_t data_in[1];

	nrf24_cs(nrf, 0);
	spi_transceive(nrf->spi_dev, data_out, ARRAY_SIZE(data_out),
		data_in, ARRAY_SIZE(data_in));
	nrf24_cs(nrf, 1);

	nrf->status = data_in[0];
	return data_in[0];
}
