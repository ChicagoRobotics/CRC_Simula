/***************************************************
Uses: Implementation of the Analog Distance Sensors following the
Distance Sensors API.

This file is designed for the Simula project by Chicago Robotics Corp.
http://www.chicagorobotics.net/products

Copyright (c) 2018, Chicago Robotics Corp.
See README.md for license details
****************************************************/

#ifndef _CRC_IR_ANALOGDISTANCE_h
#define _CRC_IR_ANALOGDISTANCE_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "CRC_DistanceSensor.h"

class CRC_IR_AnalogDistance : public CRC_DistanceSensor {
public:
	CRC_IR_AnalogDistance(int activationPin, int readingPin);
	double readDistance();
};

#endif
