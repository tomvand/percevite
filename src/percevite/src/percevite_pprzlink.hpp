#ifndef PERCEVITE_PPRZLINK_HPP__
#define PERCEVITE_PPRZLINK_HPP__

#define DOWNLINK 1

#define USE_UDP0 TRUE
#define UDP0_HOST 192.168.42.1 // Bebop2 static IP
#define UDP0_PORT_OUT 4244
#define UDP0_PORT_IN 4245
#define UDP0_BROADCAST TRUE

#include "pprzlink/pprz_transport.h"
#include "pprzlink/messages.h"
#include "udp.h"

#endif
