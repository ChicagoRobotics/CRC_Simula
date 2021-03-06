/***************************************************
Uses: General purpose logger abstraction. For now, just logs to the specified 
Print outputs.

This file is designed for the Simula project by Chicago Robotics Corp.
http://www.chicagorobotics.net/products

Copyright (c) 2018, Chicago Robotics Corp.
See README.md for license details
****************************************************/

#ifndef _CRC_Logger_h
#define _CRC_Logger_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Print.h>

class CRC_LoggerClass
{
 protected:
	 boolean _initialized;
	 uint8_t _currentLevel;

	 Print *			_destinations[2];
	 uint8_t			_totalDestinations;
	 inline void		dispatch(char * message);
	 inline void        dispatch(const __FlashStringHelper* message);

	 char _text[255];   // Logging of formatted messages
 public:
	 CRC_LoggerClass();

	static const uint8_t LOG_TRACE = 0;
	static const uint8_t LOG_INFO = 1;
	static const uint8_t LOG_WARN = 2;
	static const uint8_t LOG_ERROR = 3;

	void addLogDestination(Print *destination);
	void setLevel(uint8_t level) { _currentLevel = level; }

	void log(uint8_t level, char* message);
	void log(uint8_t level, const char* message);
	void log(uint8_t level, const __FlashStringHelper* message);
	void logF(uint8_t level, const char* format, ...);
	void logF(uint8_t level, const  __FlashStringHelper* format, ...);

};

extern CRC_LoggerClass crcLogger;

#endif

