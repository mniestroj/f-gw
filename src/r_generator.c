#include "common.h"
#include "r_generator.h"

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

void toggle_r_generation(void)
{
	if (k_timer_remaining_get(&event_generation_timer))
		k_timer_stop(&event_generation_timer);
	else
		k_timer_start(&event_generation_timer,
			K_MSEC(1000), K_MSEC(1000));
}
