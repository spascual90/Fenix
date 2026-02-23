#ifndef ORDER_H
#define ORDER_H

#include <Arduino.h>

#include "GPSfix.h"

#include "Autopilot.h"

#define YES true
#define NO false


// A class for holding a Order: type
class SERIALorder {
public:
	SERIALorder();
	virtual ~SERIALorder();

	e_actions get_order() const;
	void set_order(e_actions);
	void reset ();

	s_APinfo APinfo;
	s_PIDgain PIDgain;
	s_instParam instParam;
	s_APB APB;
	//s_HDT HDT;
	s_HDG HDG;
	s_TWD TWD;
	s_AWA AWA;
	s_IMUcal IMUcal;
	s_FBKcal FBKcal;
	s_calibrate_py calibrate_py;
	s_SOG SOG;

	//For testing purposes only
	bool TEST_isValid = false;

	// Is Valid indicates if data received is valid or not
	bool isValid=NO;
	// An information request has been received with message $PEMC,08
	bool isRequest = NO;

private:
	e_actions _order= NO_INSTRUCTION;
};

#endif
