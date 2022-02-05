/*
 * Autopilot.h
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#ifndef AUTOPILOT_H_
#define AUTOPILOT_H_

// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

////Fenix version
//#define ARDUINO_VERSION "v.3.1.B1"
//
////v.2.3.B1 implementation of capability to receive bearing from external IMU through HDM messages reception
////v.2.4.B1 implementation of capability to receive relative wind direction through VWR messages reception
////v.2.5.B2 IMU Calibration blocked in operational modes: IMU recalibration in ALL operational modes (not only STAND_BY)
////v.2.5.B2 IMU is not providing any value, keep previous value as the best approach
////v.2.6.B1 implementation of Wind Mode
////v.3.0.B1 Compatibility with Virtuino 6 (Virtuino for Fenix App.4.0) and retrocompatibility with Virtuino 5 (Virtuino for Fenix App.3.0)
////v.3.1.B1 Fix IMU not working since 2.5 (internal IMU not working, external IMU ok)
//
////DEBUG
//#define BUZZER //Comment this line to silent buzzer. SAFETY NOTICE: Only for DEBUGGING purposes!
//#define TXNMEA //Comment this line to stop periodic NMEA Transmission
////#define RESTORE_EEPROM //Uncomment this line to reset EEPROM memory
//
//// Buzzer PIN
//#define PIN_BUZZER A12
//
//#define DELAY_BUZZBEAT_TIME 50 // Buzzer beat time in msec.
//#define MAX_APB_TIME 10000 // Maximum time of APB data validity
//#define MAX_HDM_TIME 10000 // Maximum time of HDM data validity
//#define MAX_VWR_TIME 10000 // Maximum time of VWR data validity
//#define LONG_LOOP_TIME 100 // Loops to update current course and target bearing

#include "ActuatorManager.h"
#include "BearingMonitor.h"
#include "GPSport.h" // Serial NMEA IF Configuration in GPSPort.h not in Fenix.ino!

// setup status
enum e_setup_status {SETUP_OK, IMU_ERROR, FEEDBACK_ERROR};

// working status
enum e_working_status {RUNNING_OK, RUNNING_ERROR, RUN_OUT_OF_TIME};

// working modes
enum e_APmode {STAND_BY, CAL_IMU_COMPLETE, XXX_DEPRECATED, CAL_FEEDBACK, AUTO_MODE, TRACK_MODE, WIND_MODE};


// Error codes
enum e_error {
	NO_ERROR
//	,IMU_NOTFOUND
};

// Warning codes
enum e_warning {
	NO_WARNING,
	FBK_ERROR_HIGH,
	OUT_OF_COURSE,
	EE_INSTPARAM_NOTFOUND,
	EE_IMU_NOTFOUND,
	IMU_LOW,
	WP_INVALID,
	NO_WIND_DATA,
	IMU_NOTFOUND
};

// Information codes
enum e_info {
	NO_MESSAGE,
	NOT_USED,
	SETUP_INPROGRESS,
	IMU_RECAL_INPROGRESS,
	EE_PID_DEFAULT,
	TRACKMODE_AVAILABLE,
	TRACKING,
	CONFIRM_NEW_WP
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
		//This is the 0� reference for Trimming function.
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
		whole_frac magVariation; //(-45�;+45�)

		//	Heading alignment
		//Installation of magnetic compass may not be aligned with boat's steering compass, or a known transit bearing.
		whole_frac headAlign; //	(-180�, 180�)

		//	Off course alarm angle
		//Angle of Off course alarm. This alarm warns if the AP is unable to maintain its course when mode <> STANDY.
		uint8_t offcourseAlarm; //	(10�;30�).

		//minimum/maximum values read on feedback (without error protection)
		uint16_t minFeedback;
		uint16_t maxFeedback;

	} ;

	struct s_IMUcal {
		// Is Valid indicates if data is valid or not
		bool isValid=false;
		struct {bool IMUcalStatus; bool SYSstatus; bool GYROstatus; bool ACCELstatus; bool MAGNstatus; bool Xstatus; bool Ystatus; bool Zstatus;} flag;

		char IMUcalstatus;
		uint8_t SYSstatus;
		uint8_t GYROstatus;
		uint8_t ACCELstatus;
		uint8_t MAGNstatus;
		uint16_t X;
		int8_t Y;
		uint8_t Z;
	} ;

	struct s_FBKcal {
		bool isValid=false;
		struct {bool cal_minFeedback; bool cal_maxFeedback;} flag;
		uint16_t cal_minFeedback ;
		uint16_t cal_maxFeedback;
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

	struct s_WP {
		s_APB APB;
		long t0;
	} ;


	struct s_HDM {
			// Is Valid indicates if data is valid or not
			bool isValid=false;
			struct {bool HDM;} flag;

			whole_frac HDM;
	} ;

	struct s_extHeading {
		s_HDM HDM;
		long t0;
	} ;

	struct s_VWR {
		// Is Valid indicates if data is valid or not
		bool isValid=false;
		struct {bool windDirDeg; bool windDirLR;} flag;

		//	Wind direction magnitude in degrees
		whole_frac windDirDeg;

		//	Wind direction Left/Right of bow
		char windDirLR;
	} ;

	struct s_windDir {
		s_VWR VWR;
		long t0;
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
		//External compass mode
		, EXT_HEADING
		//Wind received
		, RELATIVE_WIND
		//Save to EEPROM
		, SAVE_CAL		// Save current calibration offsets to EEPROM
		, SAVE_INST		// Save current instParam to EEPROM
		, SAVE_GAIN		// Save current PIDgain to EEPROM
		, SAVE_HC	// Save HARDCODED PIDgain and instParam to EEPROM
	};

	struct  {
		// ver 1: Fenix v0.1
		// ver 2: save CHECK value before IMU, InstParam and PID structures
		int ver =2; // TODO: Version of EEPROM structure is not saved
		long Flag= 0;
		long IMU=25;
		long InstParam=73;
		long PIDgain=125;
	} EE_address;

//HARDCODED gain and installation parameters
	static s_gain const HC_GAIN ={{8,0}, {0,1}, {1,0}};
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
	bool setCurrentMode(e_APmode = STAND_BY);

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

	bool isExtHeading() const {
		return _extHeading.HDM.isValid;
	}

	float getNextCourse(void);
	void setNextCourse(float nextCourse);

	void Request_PIDgain(s_PIDgain & PIDgain);
	void Request_instParam(s_instParam & instParam);
	bool Change_instParam (s_instParam instParam);
	void Start_Stop(e_start_stop type);
	void Start_Stop_wind(void);
	void Enter_Exit_FBK_Calib(void);

	//TRACK MODE
	bool activateWPnext(void); // User push Next button
	void APBreceived(s_APB APB);

	//EXTERNAL COMPASS MODE
	void HDMreceived(s_HDM HDM);

	//WIND MODE
	void VWRreceived(s_VWR VWR);
	int getWindDir(void);

	//OVERLOADED FUNCTIONS
	int changeRudder(int delta_rudder);
    void SetTunings(double, double,       // * While most users will set the tunings once in the
                    double);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    void setHeadingDev(float headingDev = 0);
	void setDBConf (type_DBConfig status);

	//FUNCTIONAL MODULE: EEPROM
	void EEPROM_setup();
	void EEPROM_format();
	void EEsave_ReqCal (bool reqCalib);
	bool EEload_ReqCal (void);
	bool EEsave_Calib();
	e_IMU_cal_status EEload_Calib();
	bool EEsave_HCParam(); //Save HARDCODED InstParam and PIDgain
	bool EEsave_instParam(bool HC=false);
	bool EEload_instParam();
	bool EEsave_PIDgain(bool HC=false);
	bool EEload_PIDgain ();
	bool isCalMode(void);

	bool EEload_CHECK (long address);
	void EEsave_CHECK (long address);


	// FUNCTIONAL MODULE: BUZZER
	void buzzer_Error();
	void buzzer_Warning();
	void buzzer_Information();
	void buzzer_Beep();

//	const s_APB& getAPB() const {
//		return _APB;
//	}
//
//	void setAPB(const s_APB& apb);
//	void setAPB(bool isValid);
//	void setNextAPB(const s_APB& apb);
//	s_APB getAPB(void) {
//		return _APB;
//	}

	// FUNCTIONAL MODULE: IMU
	void Start_Cal();
	void Cancel_Cal();
	void Save_Cal(){
		EEsave_Calib();
		EEsave_ReqCal(false); // Update Calibration Flag to disabled
	}

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
		if (warning!=NO_WARNING) buzzer_Warning();
	}

	e_info getInformation() const {
		return _information;
	}

	void setInformation(e_info information = NO_MESSAGE) {
		if (information!=_information) buzzer_Information();
		_information = information;
		sprintf(DEBUG_buffer,"!INFORMATION Code: %i\n", _information);
		DEBUG_print();

	}

private:
	e_APmode _currentMode= STAND_BY; // current working mode
	float _targetBearing= 0; // target vessel bearing
	float _nextCourse= 0; // Next course in STDBY/ AUTO Mode
	String _status[5] = { "STAND BY", "FOLLOW BEARING", "CALIBRATING" };

	// FUNCTIONAL MODULE: WORKING MODES
	bool before_changeMode(e_APmode newMode, e_APmode currentMode);
	bool after_changeMode(e_APmode currentMode, e_APmode preMode);
	e_working_status compute_OperationalMode(void);
	e_working_status compute_Stand_By(void);
	e_working_status compute_Cal_IMU(bool completeCal);
	e_working_status compute_Cal_Feedback(void);
	void computeLongLoop(void);

	//TRACK MODE
	s_WP _WPactive, _WPnext;
	void setWPactive(s_APB APB);
	void setWPnext(s_APB APB);
	void computeLongLoop_heading(void);
	void computeLongLoop_WP(void);
	void computeLongLoop_TrackMode(void);
	void computeLongLoop_WindDir(void);

	//External compass mode
	s_extHeading _extHeading;
	void set_extHeading(s_HDM HDM);
	bool isValid_HDM (void);

	//Wind mode
	s_windDir _windDir;
	int _targetWindDir; // Target wind direction relative to heading
	void set_windDir(s_VWR VWR);
	bool isValid_VWR (void);


	void reset(){
		resetFunc();  //call reset
	}

	void(* resetFunc) (void) = 0; //declare reset function @ address 0

	// FUNCTIONAL MODULE: EEPROM
	byte CRC8(const byte *data, size_t dataLength);
	const uint8_t CHECKvalue = 170; // 170 = 10101010 in binary


	// FUNCTIONAL MODULE: BUZZER
	void buzzer_setup();
	void buzzer_IBIT();
	int get_PIN_BUZZER() {return PIN_BUZZER;}
	void buzzer_noTone();
	void buzzer_tone_start (unsigned long frequency=1000, int duration=0);
	void buzzer_play();
	unsigned long _buzzFrec = 1000;
	int _buzzDur = 0;
	void BuzzReset(void);
	bool IsBuzzTime (void);
	bool _Buzz=false;
	unsigned long _DelayBuzzStart = millis();



	// FUNCTIONAL MODULE: OFF COURSE ALARM
	uint8_t _offCourseAlarm; // off course alarm angle
	double const _offCourseMaxTime = 15000; // max off course time before alarm starts
	bool _offCourseAlarmActive=false;
	bool compute_OCA (float delta);

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
