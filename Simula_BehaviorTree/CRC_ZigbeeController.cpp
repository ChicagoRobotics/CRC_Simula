/***************************************************
Uses: Module for controlling and configuring the Zigbee module.
Currenty, this discovers and sets up the Zigbee Wifi module with
SSID roaming support. SSIDs are stored in the configuration manager.

Wireless Configuration in configuration manager:
wifi.ssid.[0-9]=<Wireless Network SSID>
wifi.mode.[0-9]=[0=No Security|1=WPA|2=WPA2|3=WEP]
wifi.psk.[0-9]=<Wireless PreShared Key>

This file is designed for the Simula project by Chicago Robotics Corp.
http://www.chicagorobotics.net/products

Copyright (c) 2017, Chicago Robotics Corp.
See README.md for license details
****************************************************/

#include "CRC_ZigbeeController.h"
#include "CRC_Logger.h"

#include "CRC_ConfigurationManager.h"
#include "CRC_Hardware.h"

// Wireless State Definitions
static const uint8_t WI_STATUS_NOTAVAILABLE = 0x00;
static const uint8_t WI_STATUS_WIFI_IDLE = 0x01;
static const uint8_t WI_STATUS_WIFI_CONNECTED = 0x02;
static const uint8_t WI_STATUS_WIFI_NOTAVAILABLE = 0x03;

static const unsigned long BAUD_RATES[] = 
{
	115200, // Preferred
	9600,   // Default
	38400,
	57600,
	230400
};

const char XBEE_CMD_OK[] PROGMEM = "OK\r";

void CRC_ZigbeeController::init(HardwareSerial & serialPort)
{
	_serialPort = &serialPort;
	_baudRate = 0; // Not Initialized. We can save/restore
	_isConnected = false;
	hardwareState.wireless = WI_STATUS_NOTAVAILABLE;

	// Have not attempted connection
	// Let's see if the xbee comes up with the existing connection
	_attemptedNetwork = -1; 
	if(!scanForModule());
}

boolean CRC_ZigbeeController::scanForModule()
{
	uint8_t readBuffer[255];
	// Attempt each Baud Rate
	int numRates = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);
	
	for (int rateId = 0; rateId < numRates; rateId++)
	{
		crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Checking for Device at Baud Rate: %lu"), BAUD_RATES[rateId]);
		_serialPort->flush();
		_serialPort->end();
		_serialPort->begin(BAUD_RATES[rateId], SERIAL_8N1);
		_serialPort->setTimeout(5000);
		if (enterCommandMode())
		{
			_baudRate = BAUD_RATES[rateId];
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Found device at: %lu"), _baudRate);
			crcLogger.log(crcLogger.LOG_INFO, F("XBEE:Identifying Device Type"));
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Device Type: %s"), sendCommand("DD", false));

			_lastAttempt.restart();

			hardwareState.wireless = WI_STATUS_WIFI_IDLE;

			// Are we connected by now?
			isConnected(true, false);

			break;
		}
	}
}

boolean CRC_ZigbeeController::enterCommandMode()
{
	char input[] = {0,0,0,0,0};

	delay(1000); // Guard Time
	_serialPort->print(F("+++")); // Enter Command Mode
	delay(1000); // Guard Time
	if (_serialPort->readBytes(input, 3) == 3)
	{
		return (strncmp_P(input, XBEE_CMD_OK, 3) == 0);
	}

	crcLogger.log(crcLogger.LOG_TRACE, F("XBEE:Timeout Waiting for Command Mode"));
	return false;	
}

void CRC_ZigbeeController::exitCommandMode()
{
	_serialPort->print(F("AT"));
	_serialPort->print(F("CN"));
	_serialPort->print(F("\r"));
	_serialPort->readStringUntil('\r');
}

char* CRC_ZigbeeController::sendCommand(char* command, boolean atomic)
{
	if (atomic) {
		enterCommandMode();
	}

	int bufferSize = sizeof(_receive);
	memset(_receive, 0, bufferSize);

	crcLogger.logF(crcLogger.LOG_TRACE, F("XBEE:Sending Command: %s"), command);
	_serialPort->print(F("AT"));
	_serialPort->print(command);
	_serialPort->print(F("\r"));
	_serialPort->readBytesUntil('\r', _receive, bufferSize);

	if (atomic) {
		exitCommandMode();
	}
	return _receive;
}

boolean CRC_ZigbeeController::isReady()
{
	if (_isConnected || !isModuleDetected()) {
		return true;
	}

	// If we gave up, stop trying
	if (hardwareState.wireless == WI_STATUS_WIFI_NOTAVAILABLE)
	{
		return false;
	}

	return connectToNetwork();
}

boolean CRC_ZigbeeController::isConnected(boolean logIsConnectedMessage, boolean atomic)
{
	if (strcmp_P(sendCommand("AI", atomic), (char *)F("0")) == 0)
	{
		_isConnected = true;

		if (logIsConnectedMessage)
		{
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Connected to Network '%s'"), getNetworkId(atomic));
		}
		hardwareState.wireless = WI_STATUS_WIFI_CONNECTED;
	}

	return _isConnected;
}

boolean CRC_ZigbeeController::connectToNetwork()
{
	// Give it time to connect
	if (_lastAttempt.elapsed() < 5000)
	{
		return _isConnected;
	}

	// Check, are we connected to a Wireless Network?
	// This could be a preconfigured network, or something we
	// Just connected to via configuration
	if (isConnected(true, true))
	{
		return _isConnected;
	}
	
	// Did we try all of them?
	if (_attemptedNetwork >= 10)
	{
		// Give up, none of our configured networks are available 
		// We're not going to keep attempting for now
		hardwareState.wireless = WI_STATUS_WIFI_NOTAVAILABLE;
		crcLogger.log(crcLogger.LOG_INFO, F("XBEE:No configured Wifi Networks available"));
		return _isConnected;
	}

	// Attempt the next network
	char temp[300];
	boolean firstAttempt = false;
	uint8_t countAttempted = 0;
	if (_attemptedNetwork <= 0)
	{
		firstAttempt = true;
	}
		
	while(_attemptedNetwork < 10)
	{
		_attemptedNetwork++;
		sprintf_P(temp, (char *) F("wifi.ssid.%d"), _attemptedNetwork);

		// Attempt each remaining index, until we reach 10
		if (crcConfigurationManager.getConfig(temp, _receive, sizeof(_receive)))
		{
			countAttempted++;

			enterCommandMode();

			// Configure SSID
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Setting SSID: %s"), _receive);
			sprintf_P(temp, (char *) F("ID %s"), _receive);
			sendCommand(temp, false);

			// Configure Encryption Mode
			sprintf_P(temp, (char *) F("wifi.mode.%d"), _attemptedNetwork);
			if (crcConfigurationManager.getConfig(temp, _receive, sizeof(_receive)))
			{
				crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Setting Mode: %s"), _receive);

				sprintf_P(temp, (char *) F("EE %s"), _receive);
				sendCommand(temp, false);
			}

			// Configure PSK Mode
			sprintf_P(temp, (char *) F("wifi.psk.%d"), _attemptedNetwork);
			if (crcConfigurationManager.getConfig(temp, _receive, sizeof(_receive)))
			{
				crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Setting PSK: %s"), _receive);

				sprintf_P(temp, (char *) F("PK %s"), _receive);
				sendCommand(temp, false);
			}

			// Apply changes and write to persistent
			sprintf_P(temp, (char *) F("AC,WR"));
			crcLogger.log(crcLogger.LOG_INFO, F("XBEE:Saving Wireless Configuration"));

			sendCommand(temp);

			exitCommandMode();

			// Start the registration timer
			_lastAttempt.restart();
			break;
		}
	}

	if (countAttempted == 0 && firstAttempt)
	{
		// Give Up, nothing was configured
		hardwareState.wireless = WI_STATUS_WIFI_NOTAVAILABLE;
		crcLogger.log(crcLogger.LOG_INFO, F("XBEE:Wifi Networks not configured"));
		return false;
	}

	return _isConnected;
}

/**
* Returns the Current Network Id
*/
char * CRC_ZigbeeController::getNetworkId(boolean atomic)
{
	return sendCommand("ID", atomic);
}