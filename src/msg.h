#include <zephyr.h>

#define MSG_PWR_ON	0x00
#define MSG_IR_ON	0x10
#define MSG_IR_OFF	0x11
#define MSG_MAG_ONOFF	0x20
#define MSG_MAG_RAW	0x22
#define MSG_STRING	0xf0

struct msg_header {
	uint32_t ts;
	uint8_t type;
} __attribute__((__packed__));

#define MSG_HEADER_LEN	(sizeof(struct msg_header))
