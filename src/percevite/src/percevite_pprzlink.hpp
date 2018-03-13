#ifndef PERCEVITE_PPRZLINK_HPP__
#define PERCEVITE_PPRZLINK_HPP__

#define PPRZLINK_BUFFER_SIZE 128

#include "pprzlink/pprz_transport.h"
#include "pprzlink/messages.h"
#include "udp.h"

class Pprzlink {
public:
	void init(void);
	void write(int length, unsigned char *bytes);
	bool read(int length, unsigned char *bytes);
private:
	struct pprz_transport trans;
	unsigned char buffer[PPRZLINK_BUFFER_SIZE];
};
extern Pprzlink pprzlink;

#endif
