// 
// 
// 

#include "CRC_HttpClient.h"
#include "CRC_Logger.h"

CRC_HttpClient::CRC_HttpClient(CRC_IP_Network & ipNetwork)
	:_ipNetwork(ipNetwork)
{

}

boolean CRC_HttpClient::isAvailable()
{
	return _ipNetwork.isAvailable();
}


void CRC_HttpClient::sendUpdate() 
{

	_ipNetwork.hasResponse();

	if (_lastSend.elapsed() < 5000) {
		return;
	}


	crcLogger.log(crcLogger.LOG_INFO, "Sending Http Update Request");

	char* data = "GET / HTTP/1.1\r\n\r\n";
	// uint32_t ipAddr = 0x0a9c8f89; // 10.156.143.137;
	IPAddress ipAddr;
	ipAddr.fromString("10.156.143.137");
	_ipNetwork.sendTcpRequest(ipAddr, 80, (uint8_t *)data, strlen(data));

	_lastSend.restart();

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