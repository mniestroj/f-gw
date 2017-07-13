#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/printk.h>
#include <misc/byteorder.h>
#include <zephyr.h>

#include <board.h>
#include <gpio.h>

#include "b_generator.h"
#include "common.h"
#include "r_generator.h"

static void button_event(void)
{
	struct event event = {
		.sensor_addr = 0x00,
		.timestamp = k_uptime_get(),
		.event_type = 1,
		.event_data = 0,
	};

	while (k_msgq_put(&event_queue, &event, K_NO_WAIT) != 0)
		k_msgq_purge(&event_queue);
}

static void button_pressed(struct device *gpio, struct gpio_callback *cb,
			u32_t pins)
{
	unsigned int i;

	for (i = SW0_GPIO_PIN; i <= SW3_GPIO_PIN; i++)
		if (pins & BIT(i))
			printk("Button %u pressed at %u\n", (unsigned int) (i - SW0_GPIO_PIN),
				(unsigned int) k_uptime_get());

	if (pins & BIT(SW0_GPIO_PIN))
		button_event();

	if (pins & BIT(SW1_GPIO_PIN))
		toggle_r_generation();

	if (pins & BIT(SW2_GPIO_PIN))
		toggle_b_generation();
}

static struct gpio_callback gpio_cb;

static void init_buttons(void)
{
	struct device *gpio;

	gpio = device_get_binding(SW0_GPIO_NAME);
	if (!gpio) {
		printk("Failed to get gpio\n");
		return;
	}

	gpio_pin_configure(gpio, SW0_GPIO_PIN,
			GPIO_DIR_IN | GPIO_PUD_PULL_UP |
			GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW);
	gpio_pin_configure(gpio, SW1_GPIO_PIN,
			GPIO_DIR_IN | GPIO_PUD_PULL_UP |
			GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW);
	gpio_pin_configure(gpio, SW2_GPIO_PIN,
			GPIO_DIR_IN | GPIO_PUD_PULL_UP |
			GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW);
	gpio_pin_configure(gpio, SW3_GPIO_PIN,
			GPIO_DIR_IN | GPIO_PUD_PULL_UP |
			GPIO_INT | GPIO_INT_EDGE | GPIO_INT_ACTIVE_LOW);
	gpio_init_callback(&gpio_cb, button_pressed,
			BIT(SW0_GPIO_PIN) | BIT(SW1_GPIO_PIN) |
			BIT(SW2_GPIO_PIN) | BIT(SW3_GPIO_PIN));
	gpio_add_callback(gpio, &gpio_cb);
	gpio_pin_enable_callback(gpio, SW0_GPIO_PIN);
	gpio_pin_enable_callback(gpio, SW1_GPIO_PIN);
	gpio_pin_enable_callback(gpio, SW2_GPIO_PIN);
	gpio_pin_enable_callback(gpio, SW3_GPIO_PIN);
}

#define BLE_STACK_SIZE	2000
#define BLE_PRIORITY	-1
K_THREAD_DEFINE(ble_tid, BLE_STACK_SIZE,
		ble_main, NULL, NULL, NULL,
		BLE_PRIORITY, K_ESSENTIAL, K_NO_WAIT);

#define NRF_STACK_SIZE	500
#define NRF_PRIORITY	1
K_THREAD_DEFINE(nrf_tid, NRF_STACK_SIZE,
		nrf_main, NULL, NULL, NULL,
		NRF_PRIORITY, K_ESSENTIAL, K_NO_WAIT);

void main(void)
{
	init_buttons();

	/* Enable r_generator by default */
	toggle_r_generation();
}
