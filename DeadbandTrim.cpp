/*
 * DeadbandTrim.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "DeadbandTrim.h"
#include <Arduino.h>

#define DB_FACTOR 3

DeadbandTrim::DeadbandTrim(type_DBConfig DBConfigStatus, type_trimConfig trimConfigStatus) {
	setDBConf(DBConfigStatus);
	setTrim(trimConfigStatus);
}

DeadbandTrim::~DeadbandTrim() {
	delete (_lr);
}

void DeadbandTrim::setTrim (type_trimConfig status) {
	_trimConfig=status;
}
void DeadbandTrim::setDBConf (type_DBConfig status){
	_DBConfig= status;
}

bool DeadbandTrim::getDeadband(int angle) {
	int abs_angle = (angle<0?-angle:angle);
	return (abs_angle<getDeadband()?true:false);
}

int DeadbandTrim::getDeadband() {
	return _deadBand;
}

float DeadbandTrim::getTrim() {
	switch (_trimConfig) {
	case TRIM_OFF:
		return 0;
	case TRIM_AUTO:
		// trimLearn already prevents too high trim values
		return _meanRudder;
	}
	return _meanRudder;
}

void DeadbandTrim::StartSampling() {
// Start sampling
delete (_lr);
_lr = new LinearRegression (0, DELAY_SAMPLING_PERIOD * NUMBER_SAMPLING);
_n=0;
_lrTime=millis();
TimerStart();
//DEBUG_PORT.print("TimerStart\n");
}

void DeadbandTrim::TimerStart() {
	_startTime = millis();
}

bool DeadbandTrim::TimerStatus (unsigned long Current) {
	// returns true if timer is still RUNNING
	// returns false if timer arrived to the limit TIME

	if ((Current -_startTime) < DELAY_SAMPLING_PERIOD) {
		return true;}

	return false;
}

int DeadbandTrim::calculateDBTrim(float PIDerrorPrima, float rudder){
	double time = millis();
	if (!TimerStatus(time)) {
		_lr->learn(time-_lrTime, PIDerrorPrima);
		trimLearn (rudder, PIDerrorPrima);

		// Start next point
		TimerStart();
		_n++;

		if (_n==NUMBER_SAMPLING) {
			// Update DB and trim
//			int l=8, d=2;
//			char c3[l+3];
//			char c4[l+3];
//			sprintf(DEBUG_buffer,"corr,stdY=%s,%s\n",dtostrf(_lr->correlation(),l,d,c3), dtostrf(DB_FACTOR *_lr->getstdY(),l,d,c4));
//			DEBUG_print();
//			DEBUG_PORT.flush();

			//Update deadband value
			double temp_deadBand = DB_FACTOR *_lr->getstdY();

			switch (_DBConfig) {
			case MINDB:
				_deadBand = VALUE_MINDB;
				break;
			case MAXDB:
				_deadBand = VALUE_MAXDB;
				break;
			case AUTODB:
				if (temp_deadBand<VALUE_MINDB) {
					_deadBand = VALUE_MINDB;
				} else if (temp_deadBand>VALUE_MAXDB) {
					_deadBand = VALUE_MAXDB;
				} else {
					_deadBand = temp_deadBand;
				}
				break;
			}

			// Restart calculations: new line
			StartSampling();
		}
	}

	return _deadBand;
}

void DeadbandTrim::trimLearn (float PIDerrorPrima, float rudder) {
	if (PIDerrorPrima<getDeadband()) { // Only updates trim value in deadband
	_meanRudder = _meanRudder + ((rudder-_meanRudder)/_trim_n);}

	// Prevents too high trim values
	if (_meanRudder > VALUE_MAXTRIM) _meanRudder = VALUE_MAXTRIM;

}
