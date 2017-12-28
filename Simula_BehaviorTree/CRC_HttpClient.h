// CRC_HttpClient.h

#ifndef _CRC_HTTPCLIENT_h
#define _CRC_HTTPCLIENT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <HardwareSerial.h>
#include "CRC_StopWatch.h"
#include "CRC_IP_Network.h"

class CRC_HttpClient
{
protected:
	CRC_IP_Network & _ipNetwork;
	CRC_StopWatch _lastSend;

public:
	CRC_HttpClient(CRC_IP_Network & ipNetwork);

	boolean isAvailable();
	void sendUpdate();
};


#endif

