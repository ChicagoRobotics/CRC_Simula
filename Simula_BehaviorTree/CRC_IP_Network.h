/***************************************************
Uses: IP Network abstraction to support arbitrary IP Services. Allows swapping out IP implementations
without impacting consumers of IP Services.

This file is designed for the Simula project by Chicago Robotics Corp.
http://www.chicagorobotics.net/products

Copyright (c) 2016, Chicago Robotics Corp.
See README.md for license details
****************************************************/
#ifndef _CRC_IP_NETWORK_h
#define _CRC_IP_NETWORK_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <IPAddress.h>

class CRC_IP_Network
{

public:
	virtual boolean isAvailable() = 0;
	virtual boolean sendTcpRequest(IPAddress &ipAddress, uint16_t port, uint8_t * content, uint16_t length) = 0;
	virtual boolean hasResponse() = 0;




};

#endif

