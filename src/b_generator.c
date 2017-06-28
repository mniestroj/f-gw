#include "b_generator.h"
#include "common.h"

enum state {
	STATE_START,
	STATE_KOSZ,
	STATE_SRODEK,
	STATE_MOTOPOMPA,
	STATE_SSAWNE,
	STATE_ROZDZIELACZ,
	STATE_POMOCNIK,
	STATE_PRZODOWNIK,
	STATE_PACHOLEK_1,
	STATE_PACHOLEK_2,
	STATE_PACHOLEK_3,
	STATE_PACHOLEK_4,
	NUM_STATES,
};

static enum state b_state;

static void b_event(enum state state)
{
	struct event event = {
		.sensor_addr = 0xB0 + state,
		.timestamp = k_uptime_get(),
		.event_type = state,
		.event_data = 0,
	};

	while (k_msgq_put(&event_queue, &event, K_NO_WAIT) != 0)
		k_msgq_purge(&event_queue);
}

static void generate_b_event(struct k_timer *timer);
K_TIMER_DEFINE(b_generation_timer, generate_b_event, NULL);

static void generate_b_event(struct k_timer *timer)
{
	static const u32_t state_timeouts[NUM_STATES] = {
		[STATE_START] = 0,
		[STATE_KOSZ] = 7000,
		[STATE_SRODEK] = 2000,
		[STATE_MOTOPOMPA] = 2000,
		[STATE_SSAWNE] = 2000,
		[STATE_ROZDZIELACZ] = 7000,
		[STATE_POMOCNIK] = 4000,
		[STATE_PRZODOWNIK] = 3000,
		[STATE_PACHOLEK_1] = 500,
		[STATE_PACHOLEK_2] = 500,
		[STATE_PACHOLEK_3] = 500,
		[STATE_PACHOLEK_4] = 500,
	};

	b_event(b_state);
	b_state++;

	if (b_state < NUM_STATES)
		k_timer_start(&b_generation_timer, K_MSEC(state_timeouts[b_state]), 0);
	else
		b_state = STATE_START;
}

void toggle_b_generation(void)
{
	if (k_timer_remaining_get(&b_generation_timer)) {
		k_timer_stop(&b_generation_timer);
	} else {
		b_state = STATE_START;
		k_timer_start(&b_generation_timer,
			K_MSEC(1), K_MSEC(0));
	}
}
