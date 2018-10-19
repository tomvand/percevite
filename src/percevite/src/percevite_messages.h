#ifndef PERCEVITE_MESSAGES_H
#define PERCEVITE_MESSAGES_H

#include <stdint.h>

enum slamdunk_flags_t {
	SD_MSG_FLAG_VECTOR        = 0x01,
	SD_MSG_FLAG_VELOCITY      = 0x02,
};

union slamdunk_to_paparazzi_msg_t {
	struct {
		uint8_t flags;         // Indicate which fields are set in the message
		float gx;              // [m] Front-right-down vector towards (sub)goal
		float gy;
		float gz;
		float vx;              // [m/s] Front-right-down velocities (in drone frame)
		float vy;
		float vz;
	};
	unsigned char bytes;
} __attribute((__packed__));
typedef union slamdunk_to_paparazzi_msg_t SlamdunkToPaparazziMsg;

union paparazzi_to_slamdunk_msg_t {
	struct {
		float tx; // [m] Front-right-down vector towards target waypoint
		float ty;
		float tz;
	};
	unsigned char bytes;
} __attribute((__packed__));
typedef union paparazzi_to_slamdunk_msg_t PaparazziToSlamdunkMsg;

#endif
