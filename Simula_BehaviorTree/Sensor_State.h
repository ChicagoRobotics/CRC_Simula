// Sensor_State.h

#ifndef _SENSOR_STATE_h
#define _SENSOR_STATE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif



struct SENSOR_STATE {
	uint16_t totalRam = 0;      // Total RAM in bytes
	uint16_t freeRam = 0;       // Free RAM in bytes
	int8_t leftMotor = 0;       // -100 -> 100
	int8_t rightMotor = 0;       // -100 -> 100	
	boolean sdCard = false;      // SD Card initialized/available state
	boolean audioPlayer = false; // Audi Player state
	boolean audioPlaying = false; // Is the audio player playing
	uint8_t wireless = 0x00;     // Wireless Status
	unsigned long loopLastTimeMillis = 0; // Last Time in millis
	unsigned long loopMinTimeMillis = 0;  // Min Time in millis
	unsigned long loopMaxTimeMillis = 0;  // Max Time in millis
	boolean buttonPressed = false;  //Default to pressed condition, stopping tree execution

	//Distance sensors
	boolean irLeftCliff;		// Left cliff sensor reading
	boolean irRightCliff;		// Right cliff sensor reading
	uint8_t irLeftCM;			// Left IR CM reading
	uint8_t irLeftFrontCM;		// Left front IR CM reading
	uint8_t irFrontCM;			// Front IR CM reading
	uint8_t irRightFrontCM;		// Right front IR CM reading
	uint8_t irRightCM;			// Right IR CM reading
	uint8_t pingFrontCM;		// Front Ping CM Reading
};

extern struct SENSOR_STATE sensorState;

class Sensors {
protected:
	unsigned long lastIrPollSensors;
public:
	void init();
	void readIR();
	boolean sensorsUpdated();
};

extern Sensors sensors;

#endif

