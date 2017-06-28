#pragma once

#include <zephyr.h>

struct event {
	u32_t sensor_addr;
	u32_t timestamp;
	u32_t event_type;
	u32_t event_data;
} __packed;

extern struct k_msgq event_queue;
