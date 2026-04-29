/*
 * CurrentFeedback.cpp
 *
 *  Created on: 16 abr. 2026
 *      Author: Sergio
 */

#include "CurrentFeedback.h"

CurrentFeedback::CurrentFeedback(int a_maxCurrent) {
	ina260 = Adafruit_INA260();
	setMaxCurrent(a_maxCurrent);
}

CurrentFeedback::~CurrentFeedback() {
	// TODO Auto-generated destructor stub
}

bool CurrentFeedback::setup(bool b_ibit){
	if (!ina260.begin()) {
	    //Serial.println("Couldn't find INA260 chip");
		_status=INA_KO;
	} else {
		//Serial.println("Found INA260 chip");
		// default continuous mode
		ina260.setMode(INA260_MODE_CONTINUOUS);
		// trigger a reading in triggered mode
		//ina260.setMode(INA260_MODE_TRIGGERED);
		// set the number of samples to average
		ina260.setAveragingCount(INA260_COUNT_64); //INA260_COUNT_4);
		// set the time over which to measure the current and bus voltage
		//ina260.setVoltageConversionTime(INA260_TIME_8_244_ms);
		ina260.setCurrentConversionTime(INA260_TIME_1_1_ms);//INA260_TIME_140_us);
		if (b_ibit) _status = IBIT();
	}
	return _status;
}

bool CurrentFeedback::IBIT(){
	// Check parameters
	sprintf(DEBUG_buffer,"Current status: %i A(max:%i A)\n",getActualCurrent(),getMaxCurrent() );
	DEBUG_print();

	return INA_OK;
}

float CurrentFeedback::readCurrent() {
	_actualCurrent =ina260.readCurrent(); // current in mA.
	return _actualCurrent;
}
