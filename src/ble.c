#include <zephyr.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>

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

static ssize_t read_time(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, u16_t len, u16_t offset)
{
	static u32_t current_time;

	current_time = k_uptime_get();

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &current_time,
				sizeof(current_time));
}

static const struct bt_uuid_128 uptime_chr_uuid = BT_UUID_INIT_128(
	0xf2, 0xee, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x13,
	0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x13);

static struct bt_gatt_attr event_attrs[] = {
	BT_GATT_PRIMARY_SERVICE(&event_uuid),
	BT_GATT_CHARACTERISTIC(&event_chr_uuid.uuid,
			BT_GATT_CHRC_INDICATE),
	BT_GATT_DESCRIPTOR(&event_chr_uuid.uuid,
			BT_GATT_PERM_NONE,
			NULL, NULL, NULL),
	BT_GATT_CCC(event_ccc_cfg, event_ccc_cfg_changed),
	BT_GATT_CHARACTERISTIC(&uptime_chr_uuid.uuid,
			BT_GATT_CHRC_READ),
	BT_GATT_DESCRIPTOR(&uptime_chr_uuid.uuid,
			BT_GATT_PERM_READ,
			read_time, NULL, NULL),
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

void ble_main(void *p1, void *p2, void *p3)
{
	int err;

	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_conn_cb_register(&conn_callbacks);

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
