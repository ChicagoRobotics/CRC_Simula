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

class CRC_HttpClient
{
protected:
	HardwareSerial * _serialPort;
	CRC_StopWatch _lastSend;

public:
	void init(HardwareSerial & serialPort);


	void sendUpdate();
};


#endif

