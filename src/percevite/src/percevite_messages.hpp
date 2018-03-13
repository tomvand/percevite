#ifndef PERCEVITE_MESSAGES_HPP__
#define PERCEVITE_MESSAGES_HPP__

union MessageToPaparazzi {
	struct {
		char text[20]; // Dummy payload
	};
	unsigned char bytes[];
} __attribute((__packed__));

union MessageFromPaparazzi {
	struct {
		char text[20]; // Dummy payload
	};
	unsigned char bytes[];
} __attribute((__packed__));

#endif
