/***************************************************
Uses: Hardware abstraction layer and definitions for the
Simula Board. Provides the constant definitions for each
board revision, as well as module initializations.

This file is designed for the Simula project by Chicago Robotics Corp.
http://www.chicagorobotics.net/products

Copyright (c) 2018, Chicago Robotics Corp.
See README.md for license details
****************************************************/

#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#include "CRC_Hardware.h"
#include "CRC_Sensors.h"
#include "CRC_Logger.h"

void CRC_HardwareClass::init() {
	seedRandomGenerator();
	setupPins();
	setupSPI();
	setupI2C();
	tick();
	crcLogger.log(crcLogger.LOG_INFO, F("Hardware initialized."));
}
void CRC_HardwareClass::tick() {
	unsigned long now = millis();
	unsigned long lastCheck = 0;
	if ((lastCheck == 0) || (now > lastCheck + battCheckIntervalMs)) {
		int rawVoltage = analogRead(crcHardware.pinBatt);
		//Standard resistive voltage divider.
		//In Mainboard v3.05 and up, the resistors are both 10K, 
		//so we multiply by two.
		//Also, 6 freshly charged Amazon black NiMH batteries
		//measure in at 8.56 volts.
		hardwareState.batteryVoltage = (rawVoltage * (5.00 / 1023.00) * 2);
		if (hardwareState.batteryVoltage > lowBatteryVoltage) {
			hardwareState.batteryLow = false;
		}
	}
}
void CRC_HardwareClass::setupPins()
{
	//Set up audio amplifier and audio player
	pinMode(vs1053_dreq, INPUT);
	pinMode(vs1053_reset, OUTPUT);
	digitalWrite(vs1053_reset, LOW);

	pinMode(vs1053_dcs, OUTPUT);
	digitalWrite(vs1053_dcs, HIGH);

	pinMode(vs1053_cs, OUTPUT);
	digitalWrite(vs1053_cs, LOW);

	// SD Card
	pinMode(sdcard_cs, OUTPUT);
	digitalWrite(sdcard_cs, HIGH);

	// Setting amplifier gain low.
	pinMode(pinAmpGain0, OUTPUT);
	pinMode(pinAmpGain1, OUTPUT);
	pinMode(pinAmpEnable, OUTPUT);
	digitalWrite(pinAmpGain0, LOW);
	digitalWrite(pinAmpGain1, LOW);
	digitalWrite(pinAmpEnable, LOW);

	// 
	pinMode(pinButtonLED, OUTPUT);
	pinMode(pinButtonA, INPUT_PULLUP);
	pinMode(pinButtonB, INPUT_PULLUP);

	// Motor Pins
	pinMode(mtr1In1, OUTPUT);
	pinMode(mtr1In2, OUTPUT);
	pinMode(mtr1Enable, OUTPUT);
	pinMode(mtr2In1, OUTPUT);
	pinMode(mtr2In2, OUTPUT);
	pinMode(mtr2Enable, OUTPUT);

	// Sensor Pins
	pinMode(pinEdge1, INPUT);
	pinMode(pinEdge2, INPUT);
	pinMode(pinPerim1, INPUT);
	pinMode(pinPerim2, INPUT);
	pinMode(pinPerim3, INPUT);
	pinMode(pinPerim4, INPUT);

	//Battery pin
	pinMode(pinBatt, INPUT);

	// Ping Pins
	pinMode(pinPingTrigger, OUTPUT);
	pinMode(pinPingEcho, INPUT);
}
void CRC_HardwareClass::setupI2C()
{
	Wire.begin();
	Wire.setTimeout(500);
}
void CRC_HardwareClass::setupSPI()
{
	SPI.begin();
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV128);
}
void CRC_HardwareClass::startScanStatus(unsigned long startTime)
{
	// Scan Free Ram START
	// Keep this block as is at start of this method
	extern int __heap_start, *__brkval;
	int v;
	hardwareState.freeRam = (uint16_t)(&v - (__brkval == 0 ? (uint16_t)&__heap_start : (uint16_t)__brkval));
	// Scan Free Ram END
};
void CRC_HardwareClass::endScanStatus(unsigned long startTime)
{
	unsigned long endTime = millis();
	unsigned long loopTime = endTime - startTime;
	hardwareState.loopLastTimeMillis = loopTime; // Last Time in millis
	hardwareState.loopMinTimeMillis = min(hardwareState.loopMinTimeMillis + 1, loopTime);  // Min Time in millis
	hardwareState.loopMaxTimeMillis = max(hardwareState.loopMaxTimeMillis, loopTime);  // Max Time in millis
}
void CRC_HardwareClass::seedRandomGenerator() {
	randomSeed(analogRead(A3));  //Get voltage reading from an unused pin.
}
void CRC_HardwareClass::announceBatteryVoltage() {
	char _voltage[20];
	dtostrf(hardwareState.batteryVoltage, 4, 3, _voltage);

	if (hardwareState.batteryLow) {
		crcLogger.logF(crcLogger.LOG_WARN, F("Batteries low at %s volts."), _voltage);
	}
	else
	{
		crcLogger.logF(crcLogger.LOG_INFO, F("Batteries good at %s volts."), _voltage);
	}
}
int CRC_HardwareClass::getRandomNumberInRange(int lowest, int highest) {
	seedRandomGenerator();
	return random(lowest, highest + 1);
}