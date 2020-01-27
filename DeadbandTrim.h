/*
 * DeadbandTrim.h
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#ifndef DEADBANDTRIM_H_
#define DEADBANDTRIM_H_

#include <LinearRegression.h>

//for DEBUGGING
#include "libraries_ext\GPSport.h"

// Period definition
#define DELAY_SAMPLING_PERIOD 1000 //mSec long period for evaluation
#define NUMBER_SAMPLING 30 // number of iterations within long period for evaluation

#define VALUE_MAXDB 25
#define VALUE_MINDB 5
#define VALUE_MAXTRIM 25

typedef enum type_DBConfig {MAXDB, MINDB, AUTODB} type_DBConfig;
typedef enum type_trimConfig {TRIM_OFF, TRIM_AUTO} type_trimConfig;

class DeadbandTrim {
public:
	DeadbandTrim(type_DBConfig DBConfigStatus = MINDB, type_trimConfig trimConfigStatus=TRIM_OFF);
	virtual ~DeadbandTrim();

	void setTrim (type_trimConfig status);
	void setDBConf (type_DBConfig status);
	type_DBConfig getDBConf (void) const {return _DBConfig;}

	void StartSampling();
	int calculateDBTrim(float PIDerrorPrima, float rudder);
	int getDeadband();
	bool getDeadband(int angle);
	float getTrim();

private:
	// Config status variables
	type_DBConfig _DBConfig = MINDB;
	type_trimConfig _trimConfig = TRIM_OFF;

	// Start monitoring period for deadband/trim factor recalculus
	void TimerStart();
	int _deadBand = 0;
	unsigned long _startTime, _lrTime;
	char _n=0;
	// Evaluates if monitoring time period is due
	bool TimerStatus (unsigned long);
	LinearRegression *_lr;


	//trimm
	float _meanRudder=0;
	int _trim_n = 20;
	void trimLearn (float PIDerrorPrima, float rudder);

};

#endif /* DEADBANDTRIM_H_ */
