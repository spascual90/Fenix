/*
 * ActuatorManager.h
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#ifndef ACTUATORCONTROLLER_H_
#define ACTUATORCONTROLLER_H_

#include <Arduino.h>
#include "GPSport.h" // Serial NMEA IF Configuration in GPSPort.h not in Fenix.ino!

//TODO: Config control


// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

//#define PIN_PWM 6
//#define PIN_DIR 7
//#define MIN_SPEED 0
//#define MAX_SPEED 255

// setup status
enum e_dir {EXTEND, RETRACT};

// ACTUATOR PARAMETERS
// PROTECT LINEAR ACTUATOR FROM HIGH THROUGHPUT DUE TO CHANGE OF DIRECTION
   #define DELAY_DIRCHANGE 500 //delay after change of dir in ms

class ActuatorController {
public:
	ActuatorController();
	virtual ~ActuatorController();

	int setSpeed (int);
	e_dir setDir (e_dir);

	e_dir getDir() const {
		return _currentDirection;
	}

	int getSpeed() const {
		return _currentSpeed;
	}
	int get_PIN_PWM() {return PIN_PWM;}
	int get_PIN_DIR() {return PIN_DIR;}


protected:
	void setup();

	//Feedback Calibration
	int cal_FBK_move(e_dir dir);

private:
	e_dir _currentDirection= EXTEND; // EXTEND or RETRACT
	int _currentSpeed=0; //Value speed between 0 and 255
	//bool blocked_dirCharge = false;
	//const unsigned long delay_dirChange;

	void IBIT();
};

#endif /* ACTUATORCONTROLLER_H_ */
