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
	DEBUG_print(F("Linear actuator... Started\n"));
	sprintf(DEBUG_buffer,"PWM,DIR=%i,%i\n",get_PIN_PWM(),get_PIN_DIR());
	DEBUG_print();
	setSpeed(MAX_SPEED);
	delay (100);
	setSpeed(MIN_SPEED);
}

int ActuatorController::cal_FBK_move(e_dir dir){
	setDir(dir);
	setSpeed(MAX_SPEED);
	delay (1000);
	setSpeed(MIN_SPEED);
	return 1;
}

e_dir ActuatorController::setDir(e_dir dir) {
	if (dir!=_currentDirection) {
		_currentDirection = dir;
		digitalWrite(PIN_DIR,_currentDirection);   // set DIR to EXTEND or RETRACT
	}

	return _currentDirection;
}

int ActuatorController::setSpeed(int speed) {
	if ((speed >=MIN_SPEED and speed<=MAX_SPEED) and (speed!=_currentSpeed)) {
		_currentSpeed = speed;
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

