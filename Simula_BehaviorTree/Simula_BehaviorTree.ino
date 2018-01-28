/*
 Name:		Simula_BehaviorTree.ino
 Created:	6/11/2016 2:05:02 PM
 Author:	jlaing
*/

#include "CRC_IP_Network.h"
#include "CRC_Simulation.h"
#include "CRC_AudioManager.h"
#include "CRC_PCA9635.h"
#include "CRC_Lights.h"
#include "CRC_Hardware.h"
#include "CRC_Sensors.h"
#include "BehaviorTree.h"
#include "CRC_PingDistance.h"
#include "CRC_IR_BinaryDistance.h"
#include "CRC_IR_AnalogDistance.h"
#include "CRC_DistanceSensor.h"
#include "CRC_Motor.h"
#include "CRC_Logger.h"
#include "CRC_ConfigurationManager.h"
#include "CRC_ZigbeeController.h"
#include "CRC_HttpClient.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <StandardCplusplus.h>
#include <Adafruit_LSM9DS0.h>	//Download from https://github.com/adafruit/Adafruit_LSM9DS0_Library/archive/master.zip
#include <Adafruit_Sensor.h>	//Download from https://github.com/adafruit/Adafruit_Sensor/archive/master.zip

Sd2Card card;
SdVolume volume;
SdFile root;
File file;

struct HARDWARE_STATE hardwareState;

CRC_Sensors sensors;
CRC_HardwareClass hardware;
CRC_SimulationClass simulation;
CRC_Motor motorLeft(hardware.enc1A, hardware.enc1B, hardware.mtr1Enable, hardware.mtr1In1, hardware.mtr1In2);
CRC_Motor motorRight(hardware.enc2A, hardware.enc2B, hardware.mtr2Enable, hardware.mtr2In1, hardware.mtr2In2);
CRC_Motors motors;
CRC_LightsClass crcLights(hardware.i2cPca9635Left, hardware.i2cPca9635Right);
CRC_AudioManagerClass crcAudio;
CRC_LoggerClass crcLogger;
CRC_ConfigurationManagerClass crcConfigurationManager;
CRC_ZigbeeController crcZigbeeWifi;
CRC_HttpClient httpClient(crcZigbeeWifi);
String robotId = "";

Behavior_Tree behaviorTree;
Behavior_Tree::Selector selector[3];
Behavior_Tree::RandomSelector randomSort[1];
Button_Stop buttonStop;
Battery_Check batteryCheck;
Cliff_Center cliffCenter;
Cliff_Left cliffLeft;
Cliff_Right cliffRight;
Perimeter_Center perimeterCenter;
Perimeter_Left perimeterLeft;
Perimeter_Right perimeterRight;
Orientation_Check orientationCheck;
Forward_Random forwardRandom(20);
Turn_Random turnLeft(15, true), turnRight(15, false);
Do_Nothing doNothing(80);

void setup() {
	Serial.begin(115200);
	crcLogger.addLogDestination(&Serial); // Log to Serial port
	crcLogger.log(crcLogger.LOG_INFO, F("Booting Simula."));
	
	//Visualize the tree here: https://www.gliffy.com/go/publish/10755293
	initializeSystem();
	behaviorTree.setRootChild(&selector[0]);
	selector[0].addChildren({ &buttonStop, &batteryCheck, &orientationCheck, &selector[1], &randomSort[0] });
	selector[1].addChildren({ &perimeterCenter, &perimeterLeft, &perimeterRight, &cliffCenter, &cliffLeft, &cliffRight });
	randomSort[0].addChildren({ &forwardRandom, &doNothing, &turnLeft, &turnRight });

	crcLights.setRandomColor();
	crcLights.showRunwayWithDelay();
	//MP3 Player & Amplifier
	crcAudio.setAmpGain(1); //0 = low, 3 = high
	crcAudio.setVolume(50, 50); //0 = loudest, 60 = softest ?
	
	//crcZigbeeWifi.init(Serial2);
	crcLogger.log(crcLogger.LOG_INFO, F("Setup complete."));

	if (!crcConfigurationManager.getConfig(F("unit.id"), robotId))
	{
		robotId = "";
	}

	if (hardwareState.sdInitialized) {
		crcAudio.playRandomAudio(F("effects/PwrUp_"), 10, F(".mp3"));
	}
}

void loop() {
	crcAudio.tick();
	simulation.tick();
	if (!buttonStop.isStopped())
	{
		sensors.imu.read();
		if (!sensors.irReadingUpdated()) {
			sensors.readIR();
		}
	}
	
	if (!behaviorTree.run()) {
		crcLogger.log(crcLogger.LOG_INFO, F("All tree nodes returned false."));
	}

	if (httpClient.isAvailable()) {
		// We can send messages up if we want to at this point.
		//httpClient.sendUpdate(robotId);
	}
}

void initializeSystem() {
	sensors.init();
	crcLogger.log(crcLogger.LOG_INFO, F("IMU initialized."));
	hardware.init();
	crcLogger.log(crcLogger.LOG_INFO, F("Hardware initialized."));
	crcLights.init();
	crcLogger.log(crcLogger.LOG_INFO, F("Lights initialized."));

	if (crcAudio.init()) {
		hardwareState.audioPlayer = true;
		crcLogger.log(crcLogger.LOG_INFO, F("Audio initialized."));
	}
	else {
		crcLogger.log(crcLogger.LOG_ERROR, F("Audio chip not detected."));
	}
	if (!sensors.imu.begin())
	{
		crcLogger.log(crcLogger.LOG_ERROR, F("Unable to detect IMU."));
	}
	else
	{
		// 1.) Set the accelerometer range
		sensors.imu.setupAccel(sensors.imu.LSM9DS0_ACCELRANGE_2G);
		// 2.) Set the magnetometer sensitivity
		sensors.imu.setupMag(sensors.imu.LSM9DS0_MAGGAIN_2GAUSS);
		// 3.) Setup the gyroscope
		sensors.imu.setupGyro(sensors.imu.LSM9DS0_GYROSCALE_245DPS);
		Serial.println(F("IMU configured."));
	}

	//Check battery voltage.
	hardware.readBatteryVoltage();
	char _voltage[20];
	dtostrf(hardwareState.batteryVoltage, 4, 2, _voltage);

	if (hardwareState.batteryLow) {
		crcLogger.logF(crcLogger.LOG_WARN, F("Batteries low at %s volts."), _voltage);
	}
	else
	{
		crcLogger.logF(crcLogger.LOG_INFO, F("Batteries good at %s volts."), _voltage);
	}

	//Initialize the motors
	motors.initialize(&motorLeft, &motorRight);

	if (!SD.begin(hardware.sdcard_cs)) {
		crcLogger.log(crcLogger.LOG_ERROR, F("SD card not detected."));
		hardwareState.sdInitialized = false;
	}
	else
	{
		crcLogger.log(crcLogger.LOG_INFO, F("SD card initialized."));
		hardwareState.sdInitialized = true;
	}
}


