/***************************************************
Uses: Provides a higher level module around the sensors on
Simula

This file is designed for the Simula project by Chicago Robotics Corp.
http://www.chicagorobotics.net/products

Copyright (c) 2018, Chicago Robotics Corp.
See README.md for license details
****************************************************/

#ifndef _CRC_SENSORS_h
#define _CRC_SENSORS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Adafruit_LSM9DS0.h>

class CRC_Sensors {
protected:
	unsigned long lastIrPollSensors;
public:
	void init();
	void activate();
	void deactivate();
	void readIR();
	boolean irReadingUpdated();
	Adafruit_LSM9DS0 imu;

	//Distance sensors
	boolean irLeftCliff = true;		// Left cliff sensor reading
	boolean irRightCliff = true;		// Right cliff sensor reading
	uint8_t irLeftCM = 0;			// Left IR CM reading
	uint8_t irLeftFrontCM = 0;		// Left front IR CM reading
	uint8_t irFrontCM = 0;			// Front IR CM reading
	uint8_t irRightFrontCM = 0;		// Right front IR CM reading
	uint8_t irRightCM = 0;			// Right IR CM reading
	uint8_t pingFrontCM = 0;		// Front Ping CM Reading
};

extern CRC_Sensors crcSensors;

#endif

