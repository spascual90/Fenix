/*
 * CurrentFeedback.h
 *
 *  Created on: 16 abr. 2026
 *      Author: Sergio
 */

#ifndef CURRENTFEEDBACK_H_
#define CURRENTFEEDBACK_H_

#include <Arduino.h>
#include <Adafruit_INA260.h>
#include "GPSport.h" // Serial NMEA IF Configuration in GPSPort.h not in Fenix.ino!

// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

#define INA_OK true
#define INA_KO false

class CurrentFeedback {
public:
	CurrentFeedback(int maxCurrent=D_MAX_CURRENT);
	virtual ~CurrentFeedback();

	float readCurrent();

	uint16_t getActualCurrent () const {
		return _actualCurrent;
	}

	int getMaxCurrent() const {
		return _max_current;
	}

	bool isCurrentMonitoring() const {
		return _status;
	}

protected:
	bool setup();
	void setMaxCurrent (int a_max_current = D_MAX_CURRENT){
		_max_current = abs(a_max_current);
		if (abs(_max_current)<1000) _max_current=1000;

	}

	;
	bool setup(bool b_ibit);
	bool IBIT();

	//void compute_Cal_Feedback();

private:
	Adafruit_INA260 ina260;
	bool _status = INA_KO;
	float _actualCurrent; //Last current value in mA
	float _max_current = D_MAX_CURRENT;

	//Values to calibrate max.current by linear actuator
	//int _cal_maxCurrent=0;

	// IBIT functions
	char* get_PIN_CURRENT() {return (PIN_CURRENT_NAME);}

};

#endif /* CURRENTFEEDBACK_H_ */
