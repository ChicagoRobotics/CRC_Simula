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
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Device Type: %s"), sendCommand(F("DD"), false));

			_lastAttempt.restart();

			hardwareState.wireless = WI_STATUS_WIFI_IDLE;

			// TODO, if we are not at Max Baud, can we bump up? This is for new / factory XBEE units
			// crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Set to bad SSID: %s"), sendCommand(F("ID blah"), true));

			// Are we connected by now?
			isConnected(true, false);

			exitCommandMode();
			break;
		}
	}
}

boolean CRC_ZigbeeController::enterCommandMode()
{
	flushInboundBuffer();
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
	flushInboundBuffer();
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
	_serialPort->flush();

	_serialPort->readBytesUntil('\r', _receive, bufferSize);

	if (atomic) {
		exitCommandMode();
	}
	return _receive;
}

char * CRC_ZigbeeController::sendCommand(const  __FlashStringHelper* command, boolean atomic = true)
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
	_serialPort->flush();

	_serialPort->readBytesUntil('\r', _receive, bufferSize);

	if (atomic) {
		exitCommandMode();
	}
	return _receive;
}

boolean CRC_ZigbeeController::isReady()
{
	if (_isConnected) {
		return true;
	}

	// If we gave up, stop trying
	if (!isModuleDetected() || hardwareState.wireless == WI_STATUS_WIFI_NOTAVAILABLE)
	{
		return false;
	}

	return connectToNetwork();
}

boolean CRC_ZigbeeController::isConnected(boolean logIsConnectedMessage, boolean atomic)
{
	char * status = sendCommand(F("AI"), atomic);
	if (strcmp_P(status, (char *)F("0")) == 0)
	{
		_isConnected = true;

		if (logIsConnectedMessage)
		{
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Connected to Network '%s'"), getNetworkId(atomic));
		}
		hardwareState.wireless = WI_STATUS_WIFI_CONNECTED;

		crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Switch to API Mode: %s"), sendCommand(F("AP 1"), atomic));

		if (!atomic) {
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE::Apply API Mode %s"), sendCommand(F("AC"), atomic));
		}

		return _isConnected;
	}
	
	crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:AI Status: %s"), status);
	if (strcmp_P(status, (char *)F("1")) == 0		// Wi-Fi transceiver initialization in progress
		    || strcmp_P(status, (char *)F("2")) == 0	// Wi-Fi transceiver initialized, but not yet scanning for access point.
			|| strcmp_P(status, (char *)F("41")) == 0	// Device joined a network and is waiting for IP configuration to complete/DHCP
			|| strcmp_P(status, (char *)F("42")) == 0	// Device is joined, IP is configured, and listening sockets are being set up.
		)
	{
		// Give it more time, it it attempting to join with current settings
		_lastAttempt.restart(); 
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
	if (isConnected(true, true) || _lastAttempt.elapsed() < 10000)
	{
		// We are connected, or need more time
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
	char temp[100];
	boolean firstAttempt = false;
	uint8_t countAttempted = 0;
	if (_attemptedNetwork < 0)
	{
		firstAttempt = true;

		// Enforce the Operational Modes we want to be in
		if (enterCommandMode())
		{
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Device Opt: %s"), sendCommand(F("DO 0"), false));
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Set Network Type: %s"), sendCommand(F("AH 2"), false));
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Set to DHCP: %s"), sendCommand(F("MA 0"), false));
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:AO Mode: %s"), sendCommand(F("AO 2"), false));
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:DL Broadcast: %s"), sendCommand(F("DL 255.255.255.255"), false));

			// For some reason setting to API Mode here messes things up. For now, init in Transparent mode
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Set to Transparent Mode: %s"), sendCommand(F("AP 0"), false));
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Apply Config: %s"), sendCommand(F("AC"), false));
			exitCommandMode();
		}
	}

	while (_attemptedNetwork < 10)
	{
		_attemptedNetwork++;
		sprintf_P(temp, (char *)F("wifi.ssid.%d"), _attemptedNetwork);

		// Attempt each remaining index, until we reach 10
		if (crcConfigurationManager.getConfig(temp, _receive, sizeof(_receive)))
		{
			countAttempted++;

			enterCommandMode();

			// Configure SSID
			sprintf_P(temp, (char *)F("ID %s"), _receive);
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Setting SSID: %s - %s"), temp, sendCommand(temp, false));

			// Configure Encryption Mode
			sprintf_P(temp, (char *)F("wifi.mode.%d"), _attemptedNetwork);
			if (crcConfigurationManager.getConfig(temp, _receive, sizeof(_receive)))
			{
				sprintf_P(temp, (char *)F("EE %s"), _receive);
				crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Setting Mode: %s - %s"), temp, sendCommand(temp, false));
			}

			// Configure PSK Mode
			sprintf_P(temp, (char *)F("wifi.psk.%d"), _attemptedNetwork);
			if (crcConfigurationManager.getConfig(temp, _receive, sizeof(_receive)))
			{
				sprintf_P(temp, (char *)F("PK %s"), _receive);
				crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Setting PSK: %s - %s"), temp, sendCommand(temp, false));
			}

			// Apply changes and 
			// TODO use AC,WR to write to persistent. But we may want to rethink writing to persistent due to roaming capability.
			sprintf_P(temp, (char *)F("AC"), _receive);
			crcLogger.logF(crcLogger.LOG_INFO, F("XBEE:Applying Wifi Config: %s - %s"), temp, sendCommand(temp, false));

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


/**
* Reads and discards all inbound bytes int he buffer
**/
void CRC_ZigbeeController::flushInboundBuffer()
{
	int c;
	char sz_temp[5];

	if (!_serialPort->available())
	{
		return;
	}

	Serial.println("Flushing Response Buffer");
	while (_serialPort->available())
	{
		c = _serialPort->read();
		sprintf(sz_temp, " %02X", c);
		Serial.print(sz_temp);
	}

	Serial.println(" ");
	Serial.println("Flush Response completed");
	
}

boolean CRC_ZigbeeController::sendTcpRequest(IPAddress &ipAddress, uint16_t port, uint8_t * content, uint16_t length)
{
	flushInboundBuffer();

	uint8_t _calcChecksum = 0x00;
	uint16_t packetLen = length + 12;

	_calcChecksum += 0x20; // IPV4 frame
	_calcChecksum += 0x01; // Frame Id
	_calcChecksum += ipAddress[0];
	_calcChecksum += ipAddress[1];
	_calcChecksum += ipAddress[2];
	_calcChecksum += ipAddress[3];
	_calcChecksum += ((uint8_t *)&port)[0];
	_calcChecksum += ((uint8_t *)&port)[1]; // dst port
	_calcChecksum += 0x01;

	for (int i = 0; i < length; i++) {
		_calcChecksum += content[i];
	}

	_calcChecksum = 0xff - _calcChecksum;



	_serialPort->write((uint8_t) 0x7E);
	_serialPort->write(&((uint8_t *)&packetLen)[1], 1); // Length MSB
	_serialPort->write(&((uint8_t *)&packetLen)[0], 1);  // Length LSB

	_serialPort->write((uint8_t)0x20); // IPV4 frame
	_serialPort->write((uint8_t)0x01); // Frame Id

	_serialPort->write(ipAddress[0]); // 4 bytes, ip address
	_serialPort->write(ipAddress[1]);
	_serialPort->write(ipAddress[2]);
	_serialPort->write(ipAddress[3]);


	_serialPort->write(&((uint8_t *)&port)[1], 1); // dst port MSB
	_serialPort->write(&((uint8_t *)&port)[0], 1);  // dst port LSB

	_serialPort->write((uint8_t)0x00); // src port MSB
	_serialPort->write((uint8_t)0x00);  // src port LSB

	// _serialPort->write(&((uint8_t *)&port)[1], 1); // src port MSB
	// _serialPort->write(&((uint8_t *)&port)[0], 1);  // src port LSB

	_serialPort->write((uint8_t)0x01); // Protocol (0-udp, 1-tcp)
	_serialPort->write((uint8_t)0x00); // Options (Leave socket open)

	_serialPort->write(content, length);
	_serialPort->write(_calcChecksum);
	_serialPort->flush();

	/**
	crcLogger.logF(crcLogger.LOG_INFO, F("Sent message IP: %d.%d.%d.%d"), ipAddress[0], ipAddress[1] , ipAddress[2], ipAddress[3]);
	crcLogger.logF(crcLogger.LOG_INFO, F("Sent message Port: %d.%d"), ((uint8_t *)&port)[1], ((uint8_t *)&port)[0]);
	crcLogger.logF(crcLogger.LOG_INFO, F("Sent message with Len (MSB): %02X"), ((uint8_t *)&packetLen)[1]);
	crcLogger.logF(crcLogger.LOG_INFO, F("Sent message with Len (LSB): %02X"), ((uint8_t *)&packetLen)[0]);
	crcLogger.logF(crcLogger.LOG_INFO, F("Sent message with Checksum: %02X"), _calcChecksum);
	**/

	return true;
}

boolean CRC_ZigbeeController::hasResponse()
{
	int c;
	char sz_temp[10];

	if (!_serialPort->available())
	{
		return;
	}

	Serial.println("Response Packet");
	while (_serialPort->available())
	{
		c = _serialPort->read();

		if (isPrintable(c) || c == '\r' || c == '\n')
		{
			Serial.print((char) c);
		}
		else
		{
			sprintf(sz_temp, " %02X ", c);
			Serial.print(sz_temp);
		}
	}
	Serial.println(" ");
	Serial.println("Finished Response");
}