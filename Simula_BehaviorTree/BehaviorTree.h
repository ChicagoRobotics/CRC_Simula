/*
Name:		BehaviorTree.h
Created:	6/11/2018 2:05:02 PM
Author:	jlaing
*/

#ifndef _BEHAVIORTREE_h
#define _BEHAVIORTREE_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "CRC_Motor.h"
#include "CRC_Sensors.h"
#include "CRC_AudioManager.h"
#include "CRC_Lights.h"
#include "CRC_Logger.h"
#include <StandardCplusplus.h>
#include <list>
#include <vector>
#include <initializer_list>
#include <algorithm>

class Behavior_Tree {  // Note:  A proper copy constructor and assignment operator should be defined, since the implicit ones use shallow copies only.
private:

public:
	class Node {  // This class represents each node in the behaviour tree.
	public:
		virtual bool run() = 0;
	};

	class CompositeNode : public Node {  //  This type of Node follows the Composite Pattern, containing a list of other Nodes.
	private:
		std::vector<Node*> children;
	public:
		const std::vector<Node*>& getChildren() const { return children; }
		void addChild(Node* child) { children.push_back(child); }
		void addChildren(std::initializer_list<Node*>&& newChildren) { for (Node* child : newChildren) addChild(child); }
		template <typename CONTAINER>
		void addChildren(const CONTAINER& newChildren) { for (Node* child : newChildren) addChild(child); }
	protected:
		std::vector<Node*> childrenShuffled() const {
			std::vector<Node*> temp = children;
			std::random_shuffle(temp.begin(), temp.end());
			return temp;
		}
	};

	class Selector : public CompositeNode {
	public:
		virtual bool run() override {
			for (Node* child : getChildren()) {  // The generic Selector implementation
				if (child->run())  // If one child succeeds, the entire operation run() succeeds.  Failure only results if all children fail.
					return true;
			}
			return false;  // All children failed so the entire run() operation fails.
		}
	};

	class RandomSelector : public CompositeNode {  //Shuffles children prior to running.
	public:
		virtual bool run() override {
			for (Node* child : childrenShuffled()) {
				if (child->run())
					return true;
			}
			return false;
		}
	};

	class Sequence : public CompositeNode {
	public:
		virtual bool run() override {
			for (Node* child : getChildren()) {  // The generic Sequence implementation.
				if (!child->run())  // If one child fails, the entire operation run() fails.  Success only results if all children succeed.
					return false;
			}
			return true;  // All children suceeded, so the entire run() operation succeeds.
		}
	};

	class Root : public Node
	{
	private:
		Node * child;
		friend class Behavior_Tree;
		void setChild(Node* newChild) { child = newChild; }
		virtual bool run() override { return child->run(); }
	};
private:
	Root * root;
public:
	Behavior_Tree() : root(new Root) {}
	void setRootChild(Node* rootChild) const { root->setChild(rootChild); }
	bool run() const { return root->run(); }
};
class Button_Gate : public Behavior_Tree::Selector {
public:
	bool isStopped() { return _gateClosed; }
	Button_Gate(int buttonNum, char* name) : _buttonNum(buttonNum), _name(name) {}
private:
	char* _name;
	bool _gateClosed = true;
	int _buttonState = HIGH;
	int _lastButtonState = HIGH;
	int _buttonNum;
	unsigned long debounceTime;
	long debounceDelay = 10;
	virtual bool run() override {
		int _reading = digitalRead(_buttonNum);
		if (_reading != _lastButtonState) {
			debounceTime = millis();
			Serial.println("Value inequal");
		}

		if (((millis() - debounceTime) > debounceDelay) && (_reading != _buttonState)) {
			_buttonState = _reading;
			if (_buttonState == HIGH) {
				_gateClosed = !_gateClosed;
				if (_gateClosed) {
					crcLogger.logF(crcLogger.LOG_INFO, F("Closed button gate %s."), _name);
				}
				else {
					crcLogger.logF(crcLogger.LOG_INFO, F("Opened button gate %s."), _name);
				}
			}
		}
		_lastButtonState = _reading;
		if (!_gateClosed) {
			for (Node* child : getChildren()) {
				child->run();
			}
			return true;
		}
	}
};
class Battery_Check : public Behavior_Tree::Node {
private:
	bool nodeActive = false;
	unsigned long now;
	unsigned long lastCheck = 0;
	int interval = 1000;
	virtual bool run() override {
		now = millis();
		if (!nodeActive) {
			if (hardwareState.batteryVoltage < crcHardware.lowBatteryVoltage) {
				crcHardware.announceBatteryVoltage();
				nodeActive = true;
				crcAudio.playRandomAudio("effects/PwrDn_", 10, ".mp3");
			}
		}
		//TODO: add ability to reactivate when batteries are good.
		return nodeActive;
	}
};
class Orientation_Check : public Behavior_Tree::Node {
private:
	bool nodeActive = false;
	const int Z_Orient_Min = 15000;
	virtual bool run() override {

		if ((!motors.active()) && (!crcAudio.isPlayingAudio()) && (crcSensors.imu.accelData.z < Z_Orient_Min)) {
			crcAudio.playRandomAudio(F("emotions/scare_"), 9, F(".mp3"));
			//Serial.print("Z: ");
			//Serial.println(sensors.lsm.accelData.z);
			nodeActive = true;
		}
		else {
			nodeActive = false;
		}
		return nodeActive;

		/*Serial.print("Accel X: "); Serial.print((int)sensors.lsm.accelData.x); Serial.print(" ");
		Serial.print("Y: "); Serial.print((int)sensors.lsm.accelData.y);       Serial.print(" ");
		Serial.print("Z: "); Serial.println((int)sensors.lsm.accelData.z);     Serial.print(" ");
		Serial.print("Mag X: "); Serial.print((int)sensors.lsm.magData.x);     Serial.print(" ");
		Serial.print("Y: "); Serial.print((int)sensors.lsm.magData.y);         Serial.print(" ");
		Serial.print("Z: "); Serial.println((int)sensors.lsm.magData.z);       Serial.print(" ");
		Serial.print("Gyro X: "); Serial.print((int)sensors.lsm.gyroData.x);   Serial.print(" ");
		Serial.print("Y: "); Serial.print((int)sensors.lsm.gyroData.y);        Serial.print(" ");
		Serial.print("Z: "); Serial.println((int)sensors.lsm.gyroData.z);      Serial.print(" ");
		Serial.print("Temp: "); Serial.print((int)sensors.lsm.temperature);    Serial.println(" ");*/
	}
};
class Cliff_Center : public Behavior_Tree::Node {
private:
	bool nodeActive = false;
	const long duration = 200;
	unsigned long currentTime;
	unsigned long nodeStartTime = 0;
	virtual bool run() override {

		currentTime = millis();

		if (!nodeActive) {
			if (crcSensors.irLeftCliff && crcSensors.irRightCliff && motors.active()) {
				nodeActive = true;
				crcLogger.logF(crcLogger.LOG_INFO, F("Cliff center detected."));
				nodeStartTime = currentTime;
				motors.setPower(-simulation.straightSpeed, -simulation.straightSpeed);
			}
		}
		else
		{
			if ((nodeStartTime + duration < currentTime) && (!crcSensors.irLeftCliff && !crcSensors.irRightCliff)) {
				crcLogger.logF(crcLogger.LOG_INFO, F("Cliff center complete."));
				nodeActive = false;
				nodeStartTime = 0;
				motors.allStop();
			}
		}
		return nodeActive;
	}
};
class Cliff_Left : public Behavior_Tree::Node {
private:
	bool nodeActive = false;
	const long backDuration = 400;
	const long turnDuration = 400;
	unsigned long currentTime;
	unsigned long nodeStartTime = 0;
	bool turnStarted = false;
	virtual bool run() override {

		currentTime = millis();

		if (!nodeActive) {
			if (crcSensors.irLeftCliff && !crcSensors.irRightCliff && motors.active()) {
				crcLogger.logF(crcLogger.LOG_INFO, F("Cliff left detected."));
				nodeStartTime = currentTime;
				nodeActive = true;
				motors.setPower(-simulation.straightSpeed, -simulation.straightSpeed);
			}
		}
		else {
			if ((nodeStartTime + backDuration < currentTime) && !turnStarted) {
				crcLogger.logF(crcLogger.LOG_INFO, F("Cliff left turning."));
				turnStarted = true;
				motors.setPower(simulation.turnSpeed, -simulation.turnSpeed);
			}
			if (nodeStartTime + backDuration + turnDuration < currentTime) {
				crcLogger.logF(crcLogger.LOG_INFO, F("Cliff left complete."));
				nodeStartTime = 0;
				motors.allStop();
				nodeActive = false;
				turnStarted = false;
			}
		}
		return nodeActive;
	}
};
class Cliff_Right : public Behavior_Tree::Node {
private:
	bool nodeActive = false;
	const long backDuration = 400;
	const long turnDuration = 400;
	unsigned long currentTime;
	unsigned long nodeStartTime = 0;
	bool turnStarted = false;
	virtual bool run() override {

		currentTime = millis();
		if (!nodeActive) {
			if (!crcSensors.irLeftCliff && crcSensors.irRightCliff && motors.active()) {
				crcLogger.logF(crcLogger.LOG_INFO, F("Cliff right detected."));
				nodeStartTime = currentTime;
				nodeActive = true;
				motors.setPower(-simulation.straightSpeed, -simulation.straightSpeed);
			}
		}
		else {
			if ((nodeStartTime + backDuration < currentTime) && !turnStarted) {
				crcLogger.logF(crcLogger.LOG_INFO, F("Cliff right turning."));
				turnStarted = true;
				motors.setPower(-simulation.turnSpeed, simulation.turnSpeed);
			}
			if (nodeStartTime + backDuration + turnDuration < currentTime) {
				crcLogger.logF(crcLogger.LOG_INFO, F("Cliff right complete."));
				nodeStartTime = 0;
				motors.allStop();
				nodeActive = false;
				turnStarted = false;
			}
		}
		return nodeActive;
	}
};
class Perimeter_Center : public Behavior_Tree::Node {
private:
	bool nodeActive = false;
	uint8_t alarmCM = 11;
	const long duration = 200;
	unsigned long currentTime;
	unsigned long nodeStartTime = 0;
	virtual bool run() override {
		currentTime = millis();
		if (!nodeActive) {
			if ((crcSensors.irFrontCM < alarmCM && crcSensors.irFrontCM > crcHardware.irMinimumCM) && (!simulation.perimeterActive)) {
				nodeStartTime = currentTime;
				nodeActive = true;
				simulation.perimeterActive = true;
				crcLogger.logF(crcLogger.LOG_INFO, F("Perimeter center activated, CM=%ul"), crcSensors.irFrontCM);
				//50% chance of turning either directon
				long randNum = random(1, 101);
				if (randNum <= 50) {
					crcLogger.logF(crcLogger.LOG_INFO, F("Turning left."));
					motors.setPower(-simulation.turnSpeed, simulation.turnSpeed);
				}
				else
				{
					crcLogger.logF(crcLogger.LOG_INFO, F("Turning right."));
					motors.setPower(simulation.turnSpeed, -simulation.turnSpeed);
				}
				return nodeActive;
			}
		}
		else {
			if ((nodeStartTime + duration < currentTime) && (crcSensors.irFrontCM >= alarmCM)) {
				crcLogger.logF(crcLogger.LOG_INFO, F("Perimeter center complete."));
				motors.allStop();
				nodeStartTime = 0;
				nodeActive = false;
				simulation.perimeterActive = false;
			}
		}
		return nodeActive;
	}
};
class Perimeter_Left : public Behavior_Tree::Node {
private:
	bool nodeActive = false;
	uint8_t alarmCM = 11;
	const long duration = 200;
	unsigned long currentTime;
	unsigned long nodeStartTime = 0;
	virtual bool run() override {
		currentTime = millis();
		if (!nodeActive) {
			if ((crcSensors.irLeftFrontCM < alarmCM && crcSensors.irLeftFrontCM > crcHardware.irMinimumCM) && !simulation.perimeterActive) {
				nodeStartTime = currentTime;
				nodeActive = true;
				simulation.perimeterActive = true;
				crcLogger.logF(crcLogger.LOG_INFO, F("Perimeter left front activated, CM=%ul"), crcSensors.irLeftFrontCM);
				motors.setPower(simulation.turnSpeed, -simulation.turnSpeed);
			}
		}
		else {
			if ((nodeStartTime + duration < currentTime) && crcSensors.irLeftFrontCM >= alarmCM) {
				crcLogger.logF(crcLogger.LOG_INFO, F("Perimeter left front complete."));
				motors.allStop();
				nodeStartTime = 0;
				nodeActive = false;
				simulation.perimeterActive = false;
			}
		}
		return nodeActive;
	}
};
class Perimeter_Right : public Behavior_Tree::Node {
private:
	bool nodeActive = false;
	uint8_t alarmCM = 11;
	const long duration = 200;
	unsigned long currentTime;
	unsigned long nodeStartTime = 0;
	virtual bool run() override {
		currentTime = millis();
		if (!nodeActive) {
			if ((crcSensors.irRightFrontCM < alarmCM && crcSensors.irRightFrontCM > crcHardware.irMinimumCM) && !simulation.perimeterActive) {
				nodeStartTime = currentTime;
				nodeActive = true;
				simulation.perimeterActive = true;
				crcLogger.logF(crcLogger.LOG_INFO, F("Perimeter right front activated, CM=%ul"), crcSensors.irRightFrontCM);
				motors.setPower(-simulation.turnSpeed, simulation.turnSpeed);
			}
		}
		else {
			if ((nodeStartTime + duration < currentTime) && crcSensors.irRightFrontCM >= alarmCM) {
				crcLogger.logF(crcLogger.LOG_INFO, F("Perimeter right front complete."));
				motors.allStop();
				nodeStartTime = 0;
				nodeActive = false;
				simulation.perimeterActive = false;
			}
		}
		return nodeActive;
	}
};
class Do_Nothing : public Behavior_Tree::Node {
	//This function is checked every checkInterval to see if it should run.
	//The interval is randomized, as is the duration of the time doing nothing.
public:
	Do_Nothing(int chance) : percentChance(chance) {}
private:
	int percentChance;

	bool nodeActive = false;
	long duration = 1000;
	unsigned long currentTime;
	unsigned long nodeStartTime = 0;

	virtual bool run() override {
		currentTime = millis();
		if (!nodeActive && !simulation.actionActive) {
			long randNum = random(1, 101);
			if (randNum <= percentChance) {
				nodeActive = true;
				simulation.actionActive = true;
				nodeStartTime = currentTime;
				crcLogger.logF(crcLogger.LOG_INFO, F("Do_Nothing active."));
			}
		}
		if (nodeActive && (nodeStartTime + duration < currentTime)) {
			crcLogger.logF(crcLogger.LOG_INFO, F("Do_Nothing complete."));
			simulation.actionActive = false;
			nodeActive = false;
			nodeStartTime = 0;
		}
		return nodeActive;
	}
};
class Forward_Random : public Behavior_Tree::Node {
	//This function is checked every checkInterval to see if it should run.
	//The interval is randomized, as is the duration of the time doing nothing.
public:
	Forward_Random(int chance) : percentChance(chance) {}
private:
	int percentChance;

	bool nodeActive = false;
	long duration;
	unsigned long currentTime;
	unsigned long nodeStartTime = 0;

	virtual bool run() override {
		currentTime = millis();
		if (!nodeActive && !simulation.actionActive) {
			long randNum = random(1, 101);
			duration = random(100, 2000);
			if (randNum <= percentChance) {
				nodeActive = true;
				simulation.actionActive = true;
				nodeStartTime = currentTime;
				crcLogger.logF(crcLogger.LOG_INFO, F("Forward_Random active, duration = %ul ms."), duration);
				motors.setPower(simulation.straightSpeed, simulation.straightSpeed);
			}
		}
		if (nodeActive && (nodeStartTime + duration < currentTime)) {
			crcLogger.logF(crcLogger.LOG_INFO, F("Forward_Random complete."));
			simulation.actionActive = false;
			nodeActive = false;
			nodeStartTime = 0;
		}
		return nodeActive;
	}
};
class Turn_Random : public Behavior_Tree::Node {
	//This function is checked every checkInterval to see if it should run.
	//The interval is randomized, as is the duration of the time doing nothing.
public:
	Turn_Random(int chance, bool clockwise) : percentChance(chance), _clockwise(clockwise) {}
private:
	int percentChance;
	bool _clockwise;
	bool nodeActive = false;
	long duration;
	unsigned long currentTime;
	unsigned long nodeStartTime = 0;

	virtual bool run() override {
		currentTime = millis();
		if (!nodeActive && !simulation.actionActive) {
			long randNum = random(1, 101);
			if (randNum <= percentChance) {
				duration = random(50, 1500);
				nodeActive = true;
				simulation.actionActive = true;
				simulation.perimeterActive = true;
				nodeStartTime = currentTime;
				crcLogger.logF(crcLogger.LOG_INFO, F("Turn_Random active, duration = %ul ms."), duration);
				if (_clockwise) {
					motors.setPower(-simulation.turnSpeed, simulation.turnSpeed);
				}
				else {
					motors.setPower(simulation.turnSpeed, -simulation.turnSpeed);
				}
			}
		}
		if (nodeActive && (nodeStartTime + duration < currentTime)) {
			crcLogger.logF(crcLogger.LOG_INFO, F("Turn_Random complete."));
			simulation.actionActive = false;
			simulation.perimeterActive = false;
			motors.allStop();
			nodeActive = false;
			nodeStartTime = 0;
		}
		return nodeActive;
	}
};

#endif

