#include "percevite_pprzlink.hpp"

#include <cstring>
#include <cassert>

Pprzlink pprzlink;

void Pprzlink::init(void) {
	pprz_transport_init(&trans);
	udp_arch_init();
}

void Pprzlink::write(int length, unsigned char *bytes) {
	pprz_msg_send_PAYLOAD(&(trans.trans_tx), &(udp0.device), AC_ID, length, bytes);
}

bool Pprzlink::read(int length, unsigned char *bytes) {
	bool message_available = false;
	pprz_check_and_parse(&(udp0.device), &trans, buffer, &message_available);
	if(message_available) {
		unsigned char msg_length = buffer[4];
		switch(buffer[3]) {
		case PPRZ_MSG_ID_PAYLOAD:
			assert(msg_length <= length);
			memcpy(bytes, &(buffer[5]), msg_length);
			break;
		default:
			break;
		}
	}
	return message_available;
}
