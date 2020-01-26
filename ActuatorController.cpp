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
	setDir(RETRACT);
	setDir(EXTEND);
	setSpeed(MAX_SPEED);
	setSpeed(MIN_SPEED);
	IBIT();
}
void ActuatorController::IBIT(){
	DEBUG_print("Linear actuator... Started\n");
	sprintf(DEBUG_buffer,"PWM,DIR=%i,%i\n",get_PIN_PWM(),get_PIN_DIR());
	DEBUG_print();
	setSpeed(MAX_SPEED);
	delay (100);
	setSpeed(MIN_SPEED);
}

void ActuatorController::cal_FBK_move(e_dir dir){
	setDir(dir);
	setSpeed(MAX_SPEED);
	delay (1000);
	setSpeed(MIN_SPEED);
}

e_dir ActuatorController::setDir(e_dir dir) {
	//static unsigned long t0;
	if (dir!=_currentDirection) {
		_currentDirection = dir;
		digitalWrite(PIN_DIR,_currentDirection);   // set DIR to EXTEND or RETRACT
		// PROTECT LINEAR ACTUATOR FROM HIGH THROUGHPUT DUE TO CHANGE OF DIRECTION
		//t0 = millis();
		//blocked_dirCharge = true;

	}
	//if ((millis()-t0) > delay_dirChange) {
	//	blocked_dirCharge = false;
	//	digitalWrite(PIN_DIR,_currentDirection);   // set DIR to EXTEND or RETRACT
	//}

	return _currentDirection;
}

int ActuatorController::setSpeed(int speed) {
	if ((speed >=MIN_SPEED and speed<=MAX_SPEED) and (speed!=_currentSpeed)) {
		_currentSpeed = speed;
		analogWrite(PIN_PWM, _currentSpeed);
	}
	/*if (!blocked_dirCharge) {
		analogWrite(PIN_PWM, _currentSpeed);
	}*/

	return _currentSpeed;
}
