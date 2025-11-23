/*
 * RudderFeedback.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "RudderFeedback.h"

RudderFeedback::RudderFeedback(int MRA, int error, int deltaCenterOfRudder, int minFeedback, int maxFeedback) {

	//setup independent variables
	setMRA(MRA, false); // TODO: MRA user configurable

	setErrorFeedback(error, false);
	setMinFeedback(minFeedback, false);
	setMaxFeedback(maxFeedback, false);
	setDeltaCenterOfRudder(deltaCenterOfRudder, false);

}

RudderFeedback::~RudderFeedback() {
	// TODO Auto-generated destructor stub
}

e_feedback_status RudderFeedback::setup(bool b_ibit){
	e_feedback_status ret = FEEDBACK_OK;
	//setup dependent variables
	setLimitMinFeedback(_min_feedback);
	setLimitMaxFeedback(_max_feedback);
	_feedback_lenght = _limit_max_feedback-_limit_min_feedback;
	_ratio = RUDDER_LENGHT/float(_feedback_lenght);
	_center_of_rudder = (RUDDER_LENGHT / 2) + 1 + getMinRudder() + getDeltaCenterOfRudder() ;
	updateCurrentRudder();

	if (b_ibit) ret = IBIT();
	return ret;
}

void RudderFeedback::setErrorFeedback (int error, bool recalc) {
	_errorFeedback = error;
	if (recalc) setup(); // recalculating dependent variables
}

void RudderFeedback::setMinFeedback(int minFeedback, bool recalc) {
	//independent variable that can be changed in Standby mode
	_min_feedback = minFeedback;
	if (recalc) setup(); // recalculating dependent variables
}

void RudderFeedback::setMaxFeedback(int maxFeedback, bool recalc) {
	//independent variable that can be changed in Standby mode
	_max_feedback = maxFeedback;
	if (recalc) setup(); // recalculating dependent variables
}

void RudderFeedback::setMRA(int MRA, bool recalc) {
	//independent variable that can be changed in Standby mode
	_MRA = MRA;
	if (recalc) setup(); // recalculating dependent variables
}


void RudderFeedback::setDeltaCenterOfRudder(int deltaCenterOfRudder, bool recalc) {
	//independent variable that can be changed in Standby mode
	_delta_center_of_rudder = deltaCenterOfRudder;
	if (recalc) setup(); // recalculating dependent variables

}

e_feedback_status RudderFeedback::IBIT(){
	int l=8, d=2;
	char c4[l+3];

	sprintf(DEBUG_buffer,"Feedback test on PIN %s ...\n", get_PIN_RUDDER());
	DEBUG_print();

	// Check feedback parameters
	updateCurrentRudder();
	sprintf(DEBUG_buffer,"Feedback status: %i (L:%i, min:%i, Max:%i)\n", getFeedback(), _feedback_lenght, getLimitMinFeedback(), getLimitMaxFeedback() );
	DEBUG_print();
	sprintf(DEBUG_buffer,"Rudder parameters: Ratio: %s\n", dtostrf(_ratio,l,d,c4));
	DEBUG_print();
	sprintf(DEBUG_buffer,"Rudder status: %i (min:%i, C:%i, Max:%i)\n",getCurrentRudder(), getMinRudder(), getCenterOfRudder(),getMaxRudder() );
	DEBUG_print();

	// Check error in the potentiometer values
	int feed_error = rudder_IBIT();
	sprintf(DEBUG_buffer,"Feedback error: %i (Max:%i)\n",feed_error, getErrorFeedback ());
	DEBUG_print();
	if (feed_error>getErrorFeedback ()) return ERROR_TOO_BIG;

	DEBUG_flush();

#ifdef VIRTUAL_ACTUATOR
	return OK_VIRTUAL;
#else
	return FEEDBACK_OK;
#endif

}

int RudderFeedback::updateCurrentRudder() {
	int Feedback = getFeedback();


	//Transforms actuator feedback into rudder metrics based on RUDDERFEEDBACK.H PARAMETERS
	_currentRudder = toRudder(Feedback-getLimitMinFeedback())+getMinRudder(); // (+MIN_RUDDER)
//	sprintf(DEBUG_buffer,"_currentRudder: %i\n", _currentRudder);
//	DEBUG_print();


//#ifdef VIRTUAL_ACTUATOR
//	_currentRudder = 0;
//#endif

	return Feedback;
}

int RudderFeedback::getFeedback() const {
#ifndef VIRTUAL_ACTUATOR
	return analogRead(PIN_RUDDER);
#else
	return _va_analogRead;
#endif
}

e_rudderStatus RudderFeedback::getRudderStatus() const {
	int status = getCurrentRudder();
	if ( abs(status) < getErrorFeedback ()) return CENTERED;
	else if (status >0) return EXTENDED;
	return RETRACTED;
}

int RudderFeedback::rudder_IBIT () {
#ifdef VIRTUAL_ACTUATOR
	return VA_ERROR;
#else
	int val, val_min=1024, val_max =0, error;
	int i = 0;
	// Corrected in V3.4-B1.1. Message was too long
	DEBUG_print(F("Checking feedback error."));
	DEBUG_print(F(" It may take a few seconds..."));

	while ( i++ < 500) {
		val = getFeedback();
		delay (10);
		if (val<val_min) val_min=val;
		if (val>val_max) val_max=val;
	}

	DEBUG_print(F("Ok.\n"));

	error = val_max-val_min;
	return error;
#endif
}

// FEEDBACK CALIBRATION FUNCTIONS
void RudderFeedback::start_calFeedback() {
	DEBUG_print(F("Feedback Calibration Mode:\n"));
	setErrorFeedback(0, false); // REMOVE ERROR PROTECTION
	setMinFeedback(D_MIN_FEEDBACK, false);
	setMaxFeedback(D_MAX_FEEDBACK, false);
	setDeltaCenterOfRudder(0, true); // launches recalculation of dependent variables
	_cal_minFeedback=D_MAX_FEEDBACK;
	_cal_maxFeedback=D_MIN_FEEDBACK;
	DEBUG_print(F("Extend then retract linear actuator...\n"));
}

void RudderFeedback::setLimitMinFeedback(int limitMinFeedback) {
	_limit_min_feedback = limitMinFeedback + 2* getErrorFeedback ();
}

void RudderFeedback::setLimitMaxFeedback(int limitMaxFeedback) {
	_limit_max_feedback = limitMaxFeedback - 2* getErrorFeedback ();
}

void RudderFeedback::getFBKcalStatus(uint16_t & cal_min, uint16_t & cal_max) {
	cal_min = _cal_minFeedback;
	cal_max = _cal_maxFeedback;
}

void RudderFeedback::compute_Cal_Feedback() {
	bool changed = false;
	int feedback = getFeedback();
	if(feedback<_cal_minFeedback) {_cal_minFeedback=feedback; changed = true;}
	if(feedback>_cal_maxFeedback) {_cal_maxFeedback=feedback; changed = true;}
}

void RudderFeedback::set_calFeedback() { //TODO: Calibration of error based on IBIT test results
	int error=2*rudder_IBIT ();
	if (error<MIN_ERROR_FEEDBACK) error = MIN_ERROR_FEEDBACK;
	sprintf(DEBUG_buffer,"Feedback error: %i\n", error );
	DEBUG_print();

	// setup new independent variables
	setErrorFeedback(error, false); // Restore error protection
	setMinFeedback(_cal_minFeedback, false);
	setMaxFeedback(_cal_maxFeedback, false);
	setDeltaCenterOfRudder(0, false);
	setup(true); // launches recalculation of dependent variables with IBIT
}
