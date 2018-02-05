/*
 Name:		Simula_BehaviorTree.ino
 Created:	6/11/2018 2:05:02 PM
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

CRC_Sensors crcSensors;
CRC_HardwareClass crcHardware;
CRC_SimulationClass simulation;
CRC_Motor motorLeft(crcHardware.enc1A, crcHardware.enc1B, crcHardware.mtr1Enable, crcHardware.mtr1In1, crcHardware.mtr1In2);
CRC_Motor motorRight(crcHardware.enc2A, crcHardware.enc2B, crcHardware.mtr2Enable, crcHardware.mtr2In1, crcHardware.mtr2In2);
CRC_Motors motors;
CRC_LightsClass crcLights(crcHardware.i2cPca9635Left, crcHardware.i2cPca9635Right);
CRC_AudioManagerClass crcAudio;
CRC_LoggerClass crcLogger;
CRC_ConfigurationManagerClass crcConfigurationManager;
CRC_ZigbeeController crcZigbeeWifi;
CRC_HttpClient httpClient(crcZigbeeWifi);
String robotId = "";

Behavior_Tree behaviorTree;
Behavior_Tree::Selector selector[1];
Behavior_Tree::Sequence sequence;
Behavior_Tree::RandomSelector randomSort;
Button_Gate buttonGateA(crcHardware.pinButtonA, "Button A"), buttonGateB(crcHardware.pinButtonB, "Button B");
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
Do_Nothing doNothing(80), doNothing2(60);

void setup() {
	Serial.begin(115200);
	crcLogger.addLogDestination(&Serial); // Log to Serial port
	crcLogger.setLevel(crcLogger.LOG_INFO);  //Set logging verbosity.
	crcLogger.log(crcLogger.LOG_INFO, F("Booting Simula."));

	//Lots of setup work here
	initializeSystem();
	
	//Behavior Tree construction. Visualize: https://www.gliffy.com/go/publish/10755293
	behaviorTree.setRootChild(&sequence);
	sequence.addChildren({ &buttonGateA, &buttonGateB });
	buttonGateA.addChildren({ &batteryCheck, &orientationCheck, &selector[0], &randomSort });
	selector[0].addChildren({ &perimeterCenter, &perimeterLeft, &perimeterRight, &cliffCenter, &cliffLeft, &cliffRight });
	randomSort.addChildren({ &forwardRandom, &doNothing, &turnLeft, &turnRight });

	//Lighting display
	crcLights.setRandomColor();
	crcLights.showRunwayWithDelay();

	//MP3 Player & Amplifier
	crcAudio.setAmpGain(1); //0 = low, 3 = high
	crcAudio.setVolume(50, 50); //0 = loudest, 60 = softest ?
	delay(2000);
	crcZigbeeWifi.init(Serial2);
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
	crcHardware.tick();
	toggleButtons();
	
	if (!behaviorTree.run()) {
		crcLogger.log(crcLogger.LOG_INFO, F("All tree nodes returned false."));
	}
}

void toggleButtons() {
	//If buttonGateA or buttonGateB are open, sensors active.
	//If buttonGateA & buttonGateB are closed, sensors deactive.

	if (buttonGateA.isClosed() && buttonGateB.isClosed()) {
		if (hardwareState.sensorsActive) {
			deactivateSensors();
		}
	}
	else {
		if (!hardwareState.sensorsActive) {
			activateSensors();
		}
		crcSensors.imu.read();
		if (!crcSensors.irReadingUpdated()) {
			crcSensors.readIR();
		}
	}

	if (!buttonGateA.isClosed() && !simulation.ledsActive()) {
		simulation.showLedBio();
	}

	if (buttonGateA.isClosed() && simulation.ledsActive()) {
		simulation.showLedNone();
	}

	if (!buttonGateB.isClosed() && httpClient.isAvailable()) {
		httpClient.sendUpdate(robotId);
	}
}

void activateSensors() {
	crcSensors.activate();
}

void deactivateSensors() {
	motors.allStop();
	crcSensors.deactivate();
	simulation.showLedNone();
	crcLights.setButtonLevel(0);
}

void initializeSystem() {
	crcHardware.init();
	crcHardware.announceBatteryVoltage();
	crcLights.init();
	crcAudio.init();
	crcSensors.init();
	if (!crcSensors.imu.begin())
	{
		crcLogger.log(crcLogger.LOG_ERROR, F("Unable to detect IMU."));
	}
	else
	{
		// 1.) Set the accelerometer range
		crcSensors.imu.setupAccel(crcSensors.imu.LSM9DS0_ACCELRANGE_2G);
		// 2.) Set the magnetometer sensitivity
		crcSensors.imu.setupMag(crcSensors.imu.LSM9DS0_MAGGAIN_2GAUSS);
		// 3.) Setup the gyroscope
		crcSensors.imu.setupGyro(crcSensors.imu.LSM9DS0_GYROSCALE_245DPS);
		Serial.println(F("IMU configured."));
	}	

	//Initialize the motors
	motors.initialize(&motorLeft, &motorRight);

	if (!SD.begin(crcHardware.sdcard_cs)) {
		crcLogger.log(crcLogger.LOG_ERROR, F("SD card not detected."));
		hardwareState.sdInitialized = false;
	}
	else
	{
		crcLogger.log(crcLogger.LOG_INFO, F("SD card initialized."));
		hardwareState.sdInitialized = true;
	}
}


