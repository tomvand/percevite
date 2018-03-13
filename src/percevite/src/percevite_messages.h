#ifndef PERCEVITE_MESSAGES_H
#define PERCEVITE_MESSAGES_H

#include <stdint.h>

union slamdunk_to_paparazzi_msg_t {
	struct {
		uint8_t safe_distance; // [dm]
	};
	unsigned char bytes;
} __attribute((__packed__));
typedef union slamdunk_to_paparazzi_msg_t SlamdunkToPaparazziMsg;

union paparazzi_to_slamdunk_msg_t {
	struct {
		char text[20]; // Dummy payload
	};
	unsigned char bytes;
} __attribute((__packed__));
typedef union paparazzi_to_slamdunk_msg_t PaparazziToSlamdunkMsg;

#endif
