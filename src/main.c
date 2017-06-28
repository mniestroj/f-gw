#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <misc/printk.h>
#include <misc/byteorder.h>
#include <zephyr.h>

#include <board.h>
#include <gpio.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

#include "b_generator.h"
#include "common.h"

#define DEVICE_NAME		CONFIG_BLUETOOTH_DEVICE_NAME
#define DEVICE_NAME_LEN		(sizeof(DEVICE_NAME) - 1)

static struct bt_uuid_128 event_uuid = BT_UUID_INIT_128(
	0xf0, 0xee, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

static struct bt_gatt_ccc_cfg event_ccc_cfg[CONFIG_BLUETOOTH_MAX_PAIRED] = {};

K_SEM_DEFINE(indicate_enabled_sem, 0, UINT_MAX);
static u8_t indicate_enabled = 0;

static u8_t indicating_event_success = 0;

static struct bt_gatt_indicate_params event_ind_params;

static void event_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	printk("event_ccc_cfg_changed %p %u\n", attr, (unsigned int) value);
	indicate_enabled = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
	k_sem_give(&indicate_enabled_sem);
}

K_SEM_DEFINE(indicating_sem, 0, UINT_MAX);
static void event_indicate_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			u8_t err)
{
	printk("Event indication %s\n", err != 0 ? "fail" : "success");
	indicating_event_success = (err ? 0 : 1);
	k_sem_give(&indicating_sem);
}

static const struct bt_uuid_128 event_chr_uuid = BT_UUID_INIT_128(
	0xf1, 0xee, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x13,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x13);

static struct bt_gatt_attr event_attrs[] = {
	BT_GATT_PRIMARY_SERVICE(&event_uuid),
	BT_GATT_CHARACTERISTIC(&event_chr_uuid.uuid,
			BT_GATT_CHRC_INDICATE),
	BT_GATT_DESCRIPTOR(&event_chr_uuid.uuid,
		        BT_GATT_PERM_NONE,
			NULL, NULL, NULL),
	BT_GATT_CCC(event_ccc_cfg, event_ccc_cfg_changed),
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      0x0d, 0x18, 0x0f, 0x18, 0x05, 0x18),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      0xf0, 0xee, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
		      0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct bt_conn *bt_connection = NULL;

static void connected(struct bt_conn *conn, u8_t err)
{
	if (err) {
		printk("Connection failed (err %u)\n", err);
	} else {
		printk("Connected\n");
		bt_connection = conn;
	}
}

static void disconnected(struct bt_conn *conn, u8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);
	bt_connection = NULL;
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	bt_gatt_register(event_attrs, ARRAY_SIZE(event_attrs));

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}

K_MSGQ_DEFINE(event_queue, sizeof(struct event), 100, 4);

static void generate_event(struct k_timer *timer)
{
	static u32_t counter = 0;
	struct event event = {
		.sensor_addr = 0xA0 + (counter % 16),
		.timestamp = k_uptime_get(),
		.event_type = 1 + (counter % 3),
		.event_data = 0,
	};
	counter++;

	while (k_msgq_put(&event_queue, &event, K_NO_WAIT) != 0)
		k_msgq_purge(&event_queue);
}
K_TIMER_DEFINE(event_generation_timer, generate_event, NULL);

static void button_event(void)
{
	static u32_t counter = 0;
	struct event event = {
		.sensor_addr = 0x00,
		.timestamp = k_uptime_get(),
		.event_type = 1,
		.event_data = 0,
	};
	counter++;

	while (k_msgq_put(&event_queue, &event, K_NO_WAIT) != 0)
		k_msgq_purge(&event_queue);
}

static void toggle_event_generation(void)
{
	if (k_timer_remaining_get(&event_generation_timer))
		k_timer_stop(&event_generation_timer);
	else
		k_timer_start(&event_generation_timer,
			K_MSEC(1000), K_MSEC(1000));
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
		toggle_event_generation();

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

void main(void)
{
	int err;

	init_buttons();

	k_timer_start(&event_generation_timer, K_MSEC(1000), K_MSEC(1000));

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_conn_cb_register(&conn_callbacks);

	/* Implement notification. At the moment there is no suitable way
	 * of starting delayed work so we do it here
	 */
	while (1) {
		struct event event;

		printk("checking queue size=%u\n", (unsigned int) k_msgq_num_used_get(&event_queue));
		while (k_msgq_get(&event_queue, &event, K_FOREVER) == 0) {
			printk("Sending new event!\n");
			do {
				int err;

				event_ind_params.attr = &event_attrs[2];
				event_ind_params.func = event_indicate_cb;
				event_ind_params.data = &event;
				event_ind_params.len = sizeof(event);

				printk("sending event with ts=%u\n", (unsigned int) event.timestamp);

			        while (!indicate_enabled)
					k_sem_take(&indicate_enabled_sem, K_FOREVER);

				if ((err = bt_gatt_indicate(bt_connection, &event_ind_params)) != 0) {
					printk("request indication error err = %d\n", err);
					continue;
				} else {
					printk("request indication ok\n");
				}

				k_sem_take(&indicating_sem, K_FOREVER);
				printk("Indication confirmed\n");
			} while (!indicating_event_success);
		}

		printk("No more events in queue?\n");
	}
}
