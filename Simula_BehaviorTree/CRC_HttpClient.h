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

	IPAddress _simulaWebIp;
	String _simulaWebHostName;
	boolean _ipLookupCompleted;
public:
	CRC_HttpClient(CRC_IP_Network & ipNetwork);

	boolean isAvailable();

	// This does not belong here, as its application specific code, so, move it out once we have it working. Make this generic HTTP Client that works with the IP Network
	// Interface
	void sendUpdate(String & robotId);
};


#endif

