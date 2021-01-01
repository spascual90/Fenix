/*
 * RudderFeedback.h
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#ifndef RUDDERFEEDBACK_H_
#define RUDDERFEEDBACK_H_

#include <Arduino.h>
#include "GPSport.h" // Serial NMEA IF Configuration in GPSPort.h not in Fenix.ino!

// #define VIRTUAL_ACTUATOR // Uncomment to simulate rudder at bow. Useful when linear actuator is not available


#define PIN_RUDDER A8
#define PIN_RUDDER_NAME "A8"

// FEEDBACK PARAMETERS HARDCODED
// Parameters for linear actuator THAT DEPENDS ON THE SPECIFIC LINEAR ACTUATOR

// MIN/MAX VALUES FOR IDEAL LINEAR ACTUATOR
#define D_MIN_FEEDBACK 0 //AS READ FROM PIN_RUDDER IN THE RETRACTED POSITION OF THE ACTUATOR
#define D_MAX_FEEDBACK 1023 //AS READ FROM PIN_RUDDER IN THE EXTENDED POSITION OF THE ACTUATOR

#define MIN_ERROR_FEEDBACK 3
#define DEFAULT_ERROR_FEEDBACK 10 //A number between 0 and 1023 to protect linear actuator from overuse

// ADDITIONAL RUDDER PARAMETERS
// Parameters for rudder
#define DEFAULT_MRA 512 // MRA = ABS_MIN_RUDDER Value in degrees *10
#define RUDDER_LENGHT 1023 // Value in degrees *10
//Rudder status
enum e_rudderStatus {EXTENDED, CENTERED, RETRACTED};
enum e_feedback_status {FEEDBACK_OK, OK_VIRTUAL, ERROR_TOO_BIG};

class RudderFeedback {
public:
	RudderFeedback(int MRA = DEFAULT_MRA, int error = DEFAULT_ERROR_FEEDBACK, int deltaCenterOfRudder=0, int minFeedback=D_MIN_FEEDBACK, int maxFeedback=D_MAX_FEEDBACK);
	virtual ~RudderFeedback();

	int getErrorFeedback () const {
		return _errorFeedback;
	}

	int getCurrentRudder() const {
		return (_currentRudder  - _center_of_rudder);
	}

	int changeRudder(int delta_rudder);

	e_rudderStatus getRudderStatus() const;

	int getFeedback() const;

	int getDeltaCenterOfRudder() const {
		return _delta_center_of_rudder;
	}


	int getCenterOfRudder() const {
		return _center_of_rudder;
	}

	int getMinRudder() const {
		return (-_MRA);
	}

	int getMaxRudder() const {
		return (_MRA-1);
	}

	int getLimitMaxFeedback() const {
		return _limit_max_feedback;
	}

	int getLimitMinFeedback() const {
		return _limit_min_feedback;
	}


	// FEEDBACK CALIBRATION FUNCTIONS
	void start_calFeedback();
	void set_calFeedback();

	void setLimitMinFeedback(int limitMinFeedback = D_MIN_FEEDBACK);
	void setLimitMaxFeedback(int limitMaxFeedback = D_MAX_FEEDBACK);
	void getFBKcalStatus(uint16_t & cal_min, uint16_t & cal_max);

protected:
	//fn to change independent variables in Standby mode
	void setDeltaCenterOfRudder(int deltaCenterOfRudder = 0, bool recalc = true);
	void setErrorFeedback (int error_feedback = DEFAULT_ERROR_FEEDBACK, bool recalc = true);
	int getMaxFeedback() const {
		return _max_feedback;
	}
	int getMinFeedback() const {
		return _min_feedback;
	}
	void setMinFeedback (int min_feedback = 0, bool recalc = true);
	void setMaxFeedback (int max_feedback = D_MAX_FEEDBACK, bool recalc = true);
	void setMRA (int MRA = DEFAULT_MRA, bool recalc = true);
	e_feedback_status setup(bool b_ibit=false); // b_ibit=true->performs IBIT at the end
	e_feedback_status IBIT();
	int updateCurrentRudder();
	int toRudder(int feedback) {
		return feedback*_ratio;
	};
	int toFeeback(int rudder) {
		return rudder/_ratio;
	}

	void compute_Cal_Feedback();


private:
	//independent variables
	int _errorFeedback = DEFAULT_ERROR_FEEDBACK;
	int _min_feedback = D_MIN_FEEDBACK;//A number between 0 and 1023.
	int _max_feedback = D_MAX_FEEDBACK;//A number between 0 and 1023.
	int _delta_center_of_rudder =0;
	int _MRA = DEFAULT_MRA; // MIN_RUDDER = -MRA; MAX_RUDDER = MRA-1

	//dependent variables
	int _limit_min_feedback = (D_MIN_FEEDBACK+DEFAULT_ERROR_FEEDBACK);//A number between 0 and 1023.
	int _limit_max_feedback = (D_MAX_FEEDBACK-DEFAULT_ERROR_FEEDBACK);//A number between 0 and 1023.
	int _feedback_lenght = (D_MAX_FEEDBACK-D_MIN_FEEDBACK-(2*DEFAULT_ERROR_FEEDBACK));
	float _ratio = RUDDER_LENGHT/_feedback_lenght;
	int _center_of_rudder;
	int _currentRudder;//Actuator feedback in rudder scale

	//Values to calibrate linear actuator
	int _cal_minFeedback=D_MAX_FEEDBACK;
	int _cal_maxFeedback=D_MIN_FEEDBACK;

	// IBIT functions
	char* get_PIN_RUDDER() {return (PIN_RUDDER_NAME);}
	int rudder_IBIT ();


};

#endif /* RUDDERFEEDBACK_H_ */
