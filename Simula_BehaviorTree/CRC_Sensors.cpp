/***************************************************
Uses: Provides a higher level module around the sensors on
Simula

This file is designed for the Simula project by Chicago Robotics Corp.
http://www.chicagorobotics.net/products

Copyright (c) 2018, Chicago Robotics Corp.
See README.md for license details
****************************************************/

#include "CRC_Sensors.h"
#include "CRC_IR_BinaryDistance.h"
#include "CRC_IR_AnalogDistance.h"
#include "CRC_PingDistance.h"
#include "CRC_Hardware.h"
#include "CRC_Logger.h"

void CRC_Sensors::init() {
	imu = Adafruit_LSM9DS0();
	crcLogger.log(crcLogger.LOG_INFO, F("IMU initialized."));
}

void CRC_Sensors::activate() {
	//Activate sensors
	digitalWrite(crcHardware.pinActEdge1, HIGH);
	digitalWrite(crcHardware.pinActEdge2, HIGH);
	digitalWrite(crcHardware.pinActPerim1, HIGH);
	digitalWrite(crcHardware.pinActPerim2, HIGH);
	digitalWrite(crcHardware.pinActPerim3, HIGH);
	digitalWrite(crcHardware.pinActPerim4, HIGH);
	digitalWrite(crcHardware.pinActFrntIR, HIGH);
	lastIrPollSensors = 0;
	hardwareState.sensorsActive = true;
}

void CRC_Sensors::deactivate() {
	//Deactivate sensors
	digitalWrite(crcHardware.pinActEdge1, LOW);
	digitalWrite(crcHardware.pinActEdge2, LOW);
	digitalWrite(crcHardware.pinActPerim1, LOW);
	digitalWrite(crcHardware.pinActPerim2, LOW);
	digitalWrite(crcHardware.pinActPerim3, LOW);
	digitalWrite(crcHardware.pinActPerim4, LOW);
	digitalWrite(crcHardware.pinActFrntIR, LOW);
	lastIrPollSensors = 0;
	hardwareState.sensorsActive = false;
}

void CRC_Sensors::readIR() {
	CRC_IR_BinaryDistance edgeLeft = CRC_IR_BinaryDistance(crcHardware.pinActEdge1, crcHardware.pinEdge1);
	CRC_IR_BinaryDistance edgeRight = CRC_IR_BinaryDistance(crcHardware.pinActEdge2, crcHardware.pinEdge2);
	CRC_IR_AnalogDistance perimLeft = CRC_IR_AnalogDistance(crcHardware.pinActPerim1, crcHardware.pinPerim1);
	CRC_IR_AnalogDistance perimLeftFront = CRC_IR_AnalogDistance(crcHardware.pinActPerim2, crcHardware.pinPerim2);
	CRC_IR_AnalogDistance perimFront = CRC_IR_AnalogDistance(crcHardware.pinActFrntIR, crcHardware.pinFrntIr);
	CRC_IR_AnalogDistance perimRightFront = CRC_IR_AnalogDistance(crcHardware.pinActPerim3, crcHardware.pinPerim3);
	CRC_IR_AnalogDistance perimRight = CRC_IR_AnalogDistance(crcHardware.pinActPerim4, crcHardware.pinPerim4);
	CRC_PingDistance frontPing = CRC_PingDistance(crcHardware.pinPingTrigger, crcHardware.pinPingEcho);

	crcSensors.irLeftCM = perimLeft.readDistance();
	crcSensors.irLeftFrontCM = perimLeftFront.readDistance();
	crcSensors.irFrontCM = perimFront.readDistance();
	crcSensors.irRightFrontCM = perimRightFront.readDistance();
	crcSensors.irRightCM = perimRight.readDistance();
	crcSensors.pingFrontCM = frontPing.readDistance();

	//If there is no object detected, then we MAY have a cliff.
	crcSensors.irLeftCliff = !edgeLeft.objectDetected();
	crcSensors.irRightCliff = !edgeRight.objectDetected();
	
	lastIrPollSensors = millis();
}

boolean CRC_Sensors::irReadingUpdated() {
	unsigned long now = millis();
	unsigned long diff = now - lastIrPollSensors;

	if (lastIrPollSensors == 0)
	{
		// First Read of Sensors - pre-debounce
		return false;
	}

	if (diff < 50)
	{
		// 50 ms is still a fresh reading
		return true;
	}

	if (diff > 1200)
	{
		Serial.println(F("Long loop, forcing sensor read."));
		return false;
		//crcLogger.logF(crcLogger.LOG_TRACE, F("Forced IR Read: %ul"), diff);
	}
	return false;
}
