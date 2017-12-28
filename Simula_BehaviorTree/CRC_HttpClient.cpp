// 
// 
// 

#include "CRC_HttpClient.h"
#include "CRC_Logger.h"

#include "CRC_ConfigurationManager.h"

#include "CRC_Sensors.h"

CRC_HttpClient::CRC_HttpClient(CRC_IP_Network & ipNetwork)
	:_ipNetwork(ipNetwork)
{
	_lastSend.restart();
	_ipLookupCompleted = false;
}

boolean CRC_HttpClient::isAvailable()
{
	// Technically, we want to do DNS Lookup when IP Services are up and running, before Http Services are available
	boolean ipNetworkIsUp = _ipNetwork.isAvailable();

	if (ipNetworkIsUp && !_ipLookupCompleted)
	{
		char szTemp[100];
		if (crcConfigurationManager.getConfig(F("simulaweb.ip"), szTemp, sizeof(szTemp)))
		{
			_ipLookupCompleted = _simulaWebIp.fromString(szTemp);
		}

		if (_ipLookupCompleted)
		{
			crcLogger.logF(crcLogger.LOG_INFO, F("CRC Web IP: %s"), szTemp);
		}

		crcConfigurationManager.getConfig(F("simulaweb.host"), _simulaWebHostName);
	}


	return ipNetworkIsUp && _ipLookupCompleted;
}


void CRC_HttpClient::sendUpdate(String & robotId) 
{

	if (robotId.length() == 0)
	{
		return; // No robot Id
	}
	_ipNetwork.hasResponse();

	// Make this configurable, global so its more obvious
	if (_lastSend.elapsed() < 10000) {
		return;
	}

	char szPostData[50];
	sprintf_P(szPostData, (char *) F("ID=%s&S1=%d.0"), robotId.c_str(), random(10, 100)); // TODO, get sensor readings using sensors

	char szTemp[255];
	sprintf_P(szTemp, (char *) F("POST /api/message HTTP/1.1\r\nHost: %s\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: %d\r\n\r\n%s\r\n"), _simulaWebHostName.c_str(), strlen(szPostData), szPostData);

	crcLogger.logF(crcLogger.LOG_TRACE, F("SEND: %s"), szTemp);
	_ipNetwork.sendTcpRequest(_simulaWebIp, 80, (uint8_t * )szTemp, strlen(szTemp));

	_lastSend.restart();
}