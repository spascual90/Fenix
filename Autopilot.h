/*
 * Autopilot.h
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#ifndef AUTOPILOT_H_
#define AUTOPILOT_H_

//DEBUG
//#define VIRTUAL_ACTUATOR // When actuator is not installed, input from feedback is random. VIRTUAL_ACTUATOR set value to minimum.
#define BUZZER //Comment this line to silent buzzer. SAFETY NOTICE: Only for DEBUGGING purposes!
//#define TXNMEA //Comment this line to stop periodic NMEA Transmission

// Buzzer PIN
#define PIN_BUZZER 8

#define MAX_APB_TIME 2000 // Maximum time of APB data validity
#define LONG_LOOP_TIME 100 // Loops to update current course and target bearing
#define MAX_LOW_QDATA 100 // Maximum iterations with low quality data from IMU

#include "ActuatorManager.h"
#include "BearingMonitor.h"
#include "GPSport.h" // Serial NMEA IF Configuration in GPSPort.h not in Fenix.ino!

// setup status
enum e_setup_status {SETUP_OK, IMU_ERROR, FEEDBACK_ERROR};

// working status
enum e_working_status {RUNNING_OK, RUNNING_ERROR, RUN_OUT_OF_TIME};

// working modes
enum e_APmode {STAND_BY, CAL_IMU, CAL_FEEDBACK, AUTO_MODE, TRACK_MODE};


// Error codes *20
enum e_error {
	NO_ERROR,
	IMU_NOTFOUND
};

// Warning codes *10
enum e_warning {
	NO_WARNING,
	FBK_ERROR_HIGH,
	IMU_RECAL_FAILED,
	EE_INSTPARAM_NOTFOUND,
	EE_IMU_NOTFOUND,
	IMU_LOW
};

// Information codes
enum e_info {
	NO_MESSAGE,
	NEW_WP_RECEIVED,
	SETUP_INPROGRESS,
	IMU_RECAL_INPROGRESS
};

//  Int8 -- 255 (-128 to +127)
//  Int16 -- (-32,768 to +32,767)
//  Int32 -- (-2,147,483,648 to +2,147,483,647)
//  Int64 -- (-9,223,372,036,854,775,808 to +9,223,372,036,854,775,807)

	struct whole_frac {
		int16_t whole;
		int16_t frac;
		void init() { whole = 0; frac = 0; };
		int32_t int32_00() const { return ((int32_t)whole) * 100L + frac; };
		int16_t int16_00() const { return whole * 100 + frac; };
		int32_t int32_000() const { return whole * 1000L + frac; };
		float float_00() const { return ((float)whole) + ((float)frac)*0.01; };
		float float_000() const { return ((float)whole) + ((float)frac)*0.001; };
		whole_frac*  Towf_00(float number) {this->whole=(int)number; this->frac=(int) (number*float(100))-(this->whole*100);return this;};
		whole_frac*  Towf_000(float number) {this->whole=(int)number; this->frac=(int) (number*1000-this->whole*1000);return this;};

	} ;

	struct s_APinfo {
		// Is Valid indicates if data is valid or not
		bool isValid = false;

		struct {bool mode; bool rudder; bool HDM; bool CTS; bool deadband; bool trim;} flag;

		//	Current Working Mode
		e_APmode mode;
		//	Current Rudder Position
		int16_t rudder;
		//	Heading Magnetic (HDM)
		whole_frac HDM;
		//	Course To Steer (CTS)
		whole_frac CTS;
		//	Deadband value
		uint8_t deadband;
		//	Trim
		whole_frac trim;
	} ;

	struct s_PIDgain_flag {
		bool Kp; bool Ki; bool Kd;} ;
	struct s_gain_flag {
		s_PIDgain_flag gain; bool sTime; bool DBConfig;} ;
	struct s_gain {
		whole_frac Kp;whole_frac Ki;whole_frac Kd;} ;



	struct s_PIDgain {
		// Is Valid indicates if data is valid or not
		bool isValid;
		s_gain gain;
		struct {s_PIDgain_flag gain; bool sTime; bool DBConfig;} flag;
		//	Sample Time
		uint16_t sTime;
		//	Deadband config (auto, min, Max)
		type_DBConfig DBConfig;
	} ;

	struct s_instParam {
		// Is Valid indicates if data is valid or not
		bool isValid;
		struct {bool centerTiller; bool maxRudder; bool avgSpeed; bool instSide; bool rudDamping; bool magVariation;bool headAlign;bool offcourseAlarm ; bool minFeedback; bool maxFeedback;} flag;

		//Centered Tiller Position:
		//Indicates the linear actuator position when the tiller is aligned with the hull.
		//This is the 0º reference for Trimming function.
		//	Centered Tiller Position
		int16_t centerTiller;

		//Maximum rudder angle:
		//Limit angle of the rudder.
		//It is used to prevent the linear actuator exceeding the rudder angle and damaging the boat.
		//Relative to Centered Tiller Position
		uint16_t maxRudder;

		//Centered Tiller and Maximum Rudder will define the MIN_FEEDBACK and MAX_FEEDBACK parameters of the linear actuator.

		//	Average Cruise Speed
		//Average Cruise Speed:
		//is the nominal speed of the boat.
		//Used by the autopilot in Track mode when speed information is not received from external sources.
		int8_t avgSpeed;

		//	Installation Side
		//Side of the boat where the linear actuator is installed.
		//STARBOARD: Turn to starboard requires extension of the linear actuator from center position. By default.
		//Linear actuator target position = Controller Output
		//
		//PORTBOARD: Turn to starboard requires retraction of the linear actuator from center position.
		//Linear actuator target position = - Controller Output
		type_instSide instSide;

		// Rudder Damping
		//Analog error in the linear actuator feedback potenciometer may cause the linear actuator to "hunt" when trying to position.
		//Rudder damping identify error in potenciometer signal ( equals ERROR_FEEDBACK).
		uint8_t rudDamping;


		//	Magnetic Variation
		//Level of magnetic variation present at the boat's current position. Used by the autopilot when information is not received from external sources.
		whole_frac magVariation; //(-45º;+45º)

		//	Heading alignment
		//Installation of magnetic compass may not be aligned with boat's steering compass, or a known transit bearing.
		whole_frac headAlign; //	(-180º, 180º)

		//	Off course alarm angle
		//Angle of Off course alarm. This alarm warns if the AP is unable to maintain its course when mode <> STANDY.
		uint8_t offcourseAlarm; //	(10º;30º).

		//minimum/maximum values read on feedback (without error protection)
		uint16_t minFeedback;
		uint16_t maxFeedback;

	} ;

	struct s_APB {
		// Is Valid indicates if data is valid or not
		bool isValid=false;
		struct {bool XTE; bool dirSteer; bool alarmCircle; bool alarmPerp; bool BOD; bool destID; bool BTW; bool CTS;} flag;

		// Cross Track Error (XTE) Magnitude - Minimum distance to route. MAGNETIC
		whole_frac XTE;
		//	Direction to steer
		char dirSteer;
		//	Arrival Circle Alarm
		char alarmCircle;
		//	Perpendicular Alarm
		char alarmPerp;
		// Bearing Origin to Next Waypoint (BOD). MAGNETIC
		whole_frac BOD;
		// Destination Waypoint ID
		char destID[5] ={'-','-','-','-','\0'};
		// Bearing, present position To Next Waypoint (BTW/BRG). MAGNETIC
		whole_frac BTW;
		// Course To Steer to Next Waypoint (CTS). MAGNETIC
		whole_frac CTS;
	} ;

	enum e_actions {
		NO_INSTRUCTION
		, START_STOP
		, INC_RUDDER_1
		, INC_RUDDER_10
		, DEC_RUDDER_1
		, DEC_RUDDER_10
		, INC_COURSE_1
		, INC_COURSE_10
		, DEC_COURSE_1
		, DEC_COURSE_10
		, NOT_AVAILABLE
		, STOP_RUDDER
		//Get Installation Parameters
		, GET_INST
		//Set Installation Parameters
		, SET_INST
		//Get PID gain
		, GET_GAIN
		//Set PID gain
		, SET_GAIN
		//Get AP Information
		, GET_APINFO
		//Start compass calibration
		, START_CAL
		//Enter/exit feedback calibration
		, EE_FBK_CAL
		//Requests
		, REQ_INST
		, REQ_GAIN
		, REQ_INFO
		//Track mode request
		, REQ_TRACK
		//Save to EEPROM
		, SAVE_CAL		// Save current calibration offsets to EEPROM
		, SAVE_INST		// Save current instParam to EEPROM
		, SAVE_GAIN		// Save current PIDgain to EEPROM
		, SAVE_HC	// Save HARDCODED PIDgain and instParam to EEPROM
	};

	struct  {
		int ver =1; // TODO: Version of EEPROM structure is not saved
		long Flag= 0;
		long IMU=25;
		long InstParam=73;
		long PIDgain=125;
	} EE_address;

//HARDCODED gain and installation parameters
	static s_gain const HC_GAIN ={{8,0}, {0,15}, {1,0}};
	static s_PIDgain_flag const HC_FLAG ={true, true, true} ;

	static s_PIDgain const HC_PIDGAIN ={
			true, //isValid
			HC_GAIN,
			HC_FLAG,true, true, // flag
			100, // sampleTime
			AUTODB} ; //DBConfig type

	static s_instParam const HC_INSTPARAM ={
			true, //isValid
			{true, true, true, true, true, true, true, true, true, true}, // flag
			0, // centerTiller
			512, //maxRudder
			5, //avdSpeed
			type_instSide::STARBOARD , //instSide
			5, //rudDamping
			{2,50}, //magVariation
			{0,0}, //headAlign
			20, //offCourseAlarm
			0, //minFeedback
			1024 // maxFeedback
	};

enum e_start_stop {CURRENT_HEADING, CURRENT_TARGET};

class Autopilot: public ActuatorManager, public Bearing_Monitor {

public:
	Autopilot(s_gain gain=HC_GAIN, int ControllerDirection=REVERSE, s_instParam ip= HC_INSTPARAM);
	virtual ~Autopilot();

	// FUNCTIONAL MODULE: WORKING MODES
	e_setup_status setup();
	e_working_status Compute();
	bool setCurrentMode(e_APmode);
	e_APmode getCurrentMode() const {
		return _currentMode;
	}
	String getCurrentModeStr() const {
		return _status[_currentMode];
	}
	float getTargetBearing() const {
		return _targetBearing;
	}
	void setTargetBearing(float targetBearing) {
		if (targetBearing<0) {targetBearing+= 360;}
		_targetBearing = fmod (targetBearing, double(360));
	}

	void Request_PIDgain(s_PIDgain & PIDgain);
	void Request_instParam(s_instParam & instParam);
	bool Change_instParam (s_instParam instParam);
	void Start_Stop(e_start_stop type);
	void Enter_Exit_FBK_Calib(void);

	float getPrevCourse() const {
		return _prevCourse;
	}

	//OVERLOADED FUNCTIONS
	int changeRudder(int delta_rudder);
    void SetTunings(double, double,       // * While most users will set the tunings once in the
                    double);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
	void setDBConf (type_DBConfig status);

	//FUNCTIONAL MODULE: EEPROM
	void EEPROM_setup();
	void EEsave_ReqCal (bool reqCalib);
	bool EEload_ReqCal (void);
	bool EEsave_Calib();
	e_IMU_status EEload_Calib();
	bool EEsave_HCParam(); //Save HARDCODED InstParam and PIDgain
	bool EEload_Param(); //Load both InstParam and PIDgain from EEPROM
	bool EEsave_instParam(bool HC=false);
	bool EEload_instParam();
	bool EEsave_PIDgain(bool HC=false);
	bool EEload_PIDgain ();
	bool isCalMode(void);

	// FUNCTIONAL MODULE: BUZZER
	void buzzer_Error();
	void buzzer_Warning();
	void buzzer_Information();
	const s_APB& getAPB() const {
		return _APB;
	}

	void setAPB(const s_APB& apb) {
		_APB = apb;
		_APBtime = millis();
	}

	// FUNCTIONAL MODULE: IMU
	void Start_Cal(){
		EEsave_ReqCal(true);// Update Calibration Flag to enabled
		resetFunc();  //call reset
	}

	void(* resetFunc) (void) = 0; //declare reset function @ address 0

	void Save_Cal(){
		EEsave_Calib();
		EEsave_ReqCal(false); // Update Calibration Flag to disabled
	}

	bool reset_calibration(void);


	//return delta value (-180, 179)
	//on error, return -360
	float delta180(float angle1, float angle2) {
	if (angle1<0 or angle1>359) return -360;
	if (angle2<0 or angle2>359) return -360;

	float angle180 = angle1- angle2;
	if (angle180>180) angle180 -=360;
	if (angle180<=-180)  angle180 +=360;
	return angle180;
	}

	// FUNCTIONAL MODULE: OFF COURSE ALARM
	bool isOffCourseAlarmActive() const {
		return _offCourseAlarmActive;
	}

// FUNCTIONAL MODULE: ERROR HANDLING

	e_error getError() const {
		return _error;
	}

	void setError(e_error error = NO_ERROR) {
		_error = error;
		sprintf(DEBUG_buffer,"!ERROR Code: %i\n", _error);
		DEBUG_print();
		buzzer_Error();
	}

	e_warning getWarning() const {
		return _warning;
	}

	void setWarning(e_warning warning = NO_WARNING) {
		_warning = warning;
		sprintf(DEBUG_buffer,"!WARNING Code: %i\n", _warning);
		DEBUG_print();
		//buzzer_Warning();
	}

	e_info getInformation() const {
		return _information;
	}

	void setInformation(e_info information = NO_MESSAGE) {
		_information = information;
		sprintf(DEBUG_buffer,"!INFORMATION Code: %i\n", _information);
		DEBUG_print();
		//buzzer_Information();

	}

private:
	e_APmode _currentMode= STAND_BY; // current working mode
	float _targetBearing= 0; // target vessel bearing
	String _status[5] = { "STAND BY", "FOLLOW BEARING", "CALIBRATING" };
	s_APB _APB; //information from plotter in Track mode
	double _APBtime;
	float _prevCourse = 0;

	//NMEA RX/TX
	bool checkAPBTimeout();

	// FUNCTIONAL MODULE: WORKING MODES
	bool before_changeMode(e_APmode newMode, e_APmode currentMode);
	bool after_changeMode(e_APmode currentMode, e_APmode preMode);
	void ComputeLongLoop(void);
	e_working_status compute_Track_Mode(void);
	e_working_status compute_Stand_By(void);
	e_working_status compute_Cal_Feedback(void);

	void setPrevCourse(float prevCourse) {
		_prevCourse = prevCourse;
	}

	// FUNCTIONAL MODULE: EEPROM
	byte CRC8(const byte *data, size_t dataLength);

	// FUNCTIONAL MODULE: BUZZER
	void buzzer_setup();
	void buzzer_IBIT();
	int get_PIN_BUZZER() {return PIN_BUZZER;}
	void buzzer_noTone();
	void buzzer_tone (unsigned long duration);

	// FUNCTIONAL MODULE: OFF COURSE ALARM
	uint8_t _offCourseAlarm; // off course alarm angle
	double const _offCourseMaxTime = 15000; // max off course time before alarm starts
	bool _offCourseAlarmActive=false;
	bool OCA_Compute (float delta);

	uint8_t getOffCourseAlarm() const {
		return _offCourseAlarm;
	}

	void setOffCourseAlarm(uint8_t offCourseAlarm) {
		_offCourseAlarm = offCourseAlarm;
	}

	// FUNCTIONAL MODULE: ERROR HANDLING
	e_info _information = NO_MESSAGE;
	e_warning _warning = NO_WARNING;
	e_error _error = NO_ERROR;


	void LongLoopReset() {
		_DelayLongLoopStart = millis();
	}

	bool IsLongLooptime () {
		// returns false if timer is ON and still RUNNING
		// returns true if timer is OFF or is ON but arrived to the limit TIME
		if ( (millis() -_DelayLongLoopStart) < LONG_LOOP_TIME ) {
			return false;}
		return true;
	}

	unsigned long _DelayLongLoopStart = millis();

};

#endif /* AUTOPILOT_H_ */
