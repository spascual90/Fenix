/*
 * ActuatorManager.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "ActuatorController.h"

ActuatorController::ActuatorController() {
	pinMode(PIN_PWM,OUTPUT);
	pinMode(PIN_DIR,OUTPUT);

}


ActuatorController::~ActuatorController() {
	// TODO Auto-generated destructor stub
}


void ActuatorController::setup(){
	//TODO: REMOVE
	setDir(RETRACT);
	setDir(EXTEND);
	setSpeed(MAX_SPEED);
	setSpeed(MIN_SPEED);
	//
	IBIT();
}
void ActuatorController::IBIT(){
	DEBUG_print(F("Linear actuator... Started\n"));
	DEBUG_sprintf("PWM,DIR",get_PIN_PWM(),get_PIN_DIR());
	cal_FBK_move(EXTEND);

}

int ActuatorController::cal_FBK_move(e_dir dir){
	digitalWrite(PIN_DIR,dir);
	analogWrite(PIN_PWM, MAX_SPEED);
	delay (1000);
	analogWrite(PIN_PWM, MIN_SPEED);
	return 1;
}

e_dir ActuatorController::setDir(e_dir dir) {
	if (dir!=_currentDirection) {
		_currentDirection = dir;
		digitalWrite(PIN_DIR,_currentDirection);   // set DIR to EXTEND or RETRACT
	}

	return _currentDirection;
}

constexpr double ALFA_05 = 0.05;
constexpr double ALFA_95 = 0.95;

// inmediate = true apply speed imediatelly
int ActuatorController::setSpeed(int speed, bool inmediate) {
	//if ((speed >=MIN_SPEED and speed<=MAX_SPEED) and (speed!=_currentSpeed)) {
	if (speed!=_currentSpeed) {
		if (inmediate) {
			_currentSpeed= speed;
		} else {
			if (speed ==0 and _currentSpeed < 30) _currentSpeed=0;
			if (speed ==MAX_SPEED and _currentSpeed > 220) _currentSpeed=MAX_SPEED;
			_currentSpeed = int (float(speed) * ALFA_05 + float (_currentSpeed) * ALFA_95);
		}
		//DEBUG_sprintf("_currentSpeed", _currentSpeed);
		analogWrite(PIN_PWM, _currentSpeed);
	}

	return _currentSpeed;
}

//VIRTUAL ACTUATOR status update
// returns a float between (-1, 1) representing the % of actuator length run in the last period
int ActuatorController::compute_VA (void) {
	float const c_ratio = (1024.0 * float(VA_SPEEDFEEDBACK)) / float(VA_LENGHTFEEDBACK);
	static long lastTime = millis();
	long  thisTime = millis();
	float deltaTime = thisTime-lastTime;
	lastTime = thisTime;
	if (_currentSpeed == 0) return 0;
	float f_value = ( c_ratio * float(deltaTime) );
		if (_currentDirection== RETRACT) f_value = -f_value;

//				int l=8, d=4;
//				char c3[l+3];
//				char c4[l+3];
//				sprintf(DEBUG_buffer,"deltaTime, value=%s,%s,%i\n",dtostrf(deltaTime,l,d,c3), dtostrf(f_value,l,d,c4), round(f_value));
//				DEBUG_print();
//				DEBUG_PORT.flush();

	return round(f_value);

}

