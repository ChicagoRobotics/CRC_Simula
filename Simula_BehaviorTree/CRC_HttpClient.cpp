// 
// 
// 

#include "CRC_HttpClient.h"
#include "CRC_Logger.h"

void CRC_HttpClient::init(HardwareSerial & serialPort)
{
	_serialPort = &serialPort;
	_lastSend.restart();
}


void CRC_HttpClient::sendUpdate() 
{

	return;

	/**

	if (_lastSend.elapsed() < 10000) {
		return;
	}

	crcLogger.log(crcLogger.LOG_INFO, "Sending Http Request");


	// HTTP GET Method first
	_serialPort->print("GET / HTTP/1.1\r\nhost: 10.156.143.137\r\n\r\n");
	_serialPort->flush();

	delay(100);

	char c;
	while (_serialPort->available())
	{
		c = _serialPort->read();
		Serial.print(c);
	}
	Serial.println();


	_lastSend.restart();
	**/
}