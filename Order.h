#ifndef ORDER_H
#define ORDER_H

#include <Arduino.h>

#include "GPSfix.h"

#include "Autopilot.h"

/*
// TYPES OF ORDER
//No order
#define NO_ORDER 0

//Switch working mode from STAND BY to AUTO. Set CTS.
#define SWITCH_MODE 1

//Increment Current Rudder by 1 Position Unit. Set STAND BY Mode.
#define INC_RUDD1 2
//Increment Current Rudder by 10 Position Unit Set STAND BY Mode.
#define INC_RUDD10 3
//Reduce Current Rudder by 1 Position Unit Set STAND BY Mode.
#define RED_RUDD1 4
//Reduce Current Rudder by 10 Position Unit Set STAND BY Mode.
#define RED_RUDD10 5

//Increment CTS by 1 Position Unit
#define INC_CTS1 6
//Increment CTS by 10 Position Unit
#define INC_CTS10 7
//Reduce CTS by 1 Position Unit
#define RED_CTS1 8
//Reduce CTS by 10 Position Unit
#define RED_CTS10 9

//Get Installation Parameters
#define GET_INST 10
//Set Installation Parameters
#define SET_INST 11

//Get PID gain
#define GET_GAIN 12
//Set PID gain
#define SET_GAIN 13

//Get AP Information
#define GET_APINFO 14

//Start compass calibration
#define SET_CAL 15

//Requests
#define REQ_INST 16
#define REQ_INFO 17
#define REQ_GAIN 18

//Auto mode request
#define REQ_NAV 19
*/

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
	s_IMUcal IMUcal;
	//s_FBKcal FBKcal;

	// Is Valid indicates if data received is valid or not
	bool isValid=NO;
	// An information request has been received with message $PEMC,08
	bool isRequest = NO;

private:
	e_actions _order= NO_INSTRUCTION;
};

#endif
