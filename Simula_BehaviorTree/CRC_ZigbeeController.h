/***************************************************
Uses: Module for controlling and configuring the Zigbee module.
Currenty, this discovers and sets up the Zigbee Wifi module with
SSID roaming support. SSIDs can be stored in the configuration manager.

This file is designed for the Simula project by Chicago Robotics Corp.
http://www.chicagorobotics.net/products

Copyright (c) 2018, Chicago Robotics Corp.
See README.md for license details
****************************************************/

#ifndef _CRC_ZIGBEECONTROLLER_h
#define _CRC_ZIGBEECONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <HardwareSerial.h>
#include <IPAddress.h>
#include "CRC_StopWatch.h"
#include "CRC_IP_Network.h"


class CRC_ZigbeeController : public CRC_IP_Network
{
 protected:
	 HardwareSerial * _serialPort;
	 unsigned long _baudRate;
	 boolean _isConnected;

	 boolean scanForModule();

	 boolean enterCommandMode();
	 void exitCommandMode();
	 
	 char _receive[255];

	 boolean connectToNetwork();
	 int8_t _attemptedNetwork;
	 CRC_StopWatch _lastAttempt;

	 // For now, we configure to talk to a single host only, eventually, support arbitrary port/transmissions
	 void flushInboundBuffer();
	 boolean isReady();
 public:
	void init(HardwareSerial & serialPort);
	inline boolean isModuleDetected() { return _baudRate > 0; }

	char * sendCommand(char * command, boolean atomic=true);
	char * sendCommand(const  __FlashStringHelper* command, boolean atomic = true);
	char * getNetworkId(boolean atomic=true);

	boolean isConnected(boolean logIsConnectedMessage, boolean atomic=true);


	// Support IP Services Interface
	boolean isAvailable() { return isReady() && _isConnected; };
	boolean sendTcpRequest(IPAddress &ipAddress, uint16_t port, uint8_t * content, uint16_t length);
	boolean hasResponse();
};

#endif

