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

#include "ActuatorManager.h"
#include "BearingMonitor.h"
#include "GPSport.h" // Serial NMEA IF Configuration in GPSPort.h not in Fenix.ino!

// setup status
enum e_setup_status {SETUP_OK, IMU_ERROR, FEEDBACK_ERROR};

// working status
enum e_working_status {RUNNING_OK, RUNNING_ERROR, RUN_OUT_OF_TIME};

// working modes
enum e_APmode {STAND_BY, CAL_IMU_COMPLETE, XXX_DEPRECATED, CAL_FEEDBACK, AUTO_MODE, TRACK_MODE, WIND_MODE, CAL_AUTOTUNE};

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
		float float_0000() const { return ((float)whole) + ((float)frac)*0.0001; };
		whole_frac*  Towf_00(float number) {this->whole=(int)number; this->frac=(int) (number*float(100))-(this->whole*100);return this;};
		whole_frac*  Towf_000(float number) {this->whole=(int)number; this->frac=(int) (number*1000-this->whole*1000);return this;};
		whole_frac*  Towf_0000(float number) {this->whole=(int)number; this->frac=(int) (number*10000-this->whole*10000);return this;};

	} ;

	struct whole_frac32 {
		int32_t whole;
		int32_t frac;
		void init() { whole = 0; frac = 0; };
//		int32_t int32_00() const { return ((int32_t)whole) * 100L + frac; };
//		int16_t int16_00() const { return whole * 100 + frac; };
//		int32_t int32_000() const { return whole * 1000L + frac; };
		float float_00000() const { return ((float)whole) + ((float)frac)*0.00001; };
		whole_frac32*  Towf_00000(float number) {this->whole=(long)number; this->frac=(long) (number*100000-this->whole*100000);return this;};

	} ;

	struct s_APinfo {
		// Is Valid indicates if data is valid or not
		bool isValid = false;

		//struct {bool mode; bool rudder; bool HDM; bool CTS; bool deadband; bool trim;} flag;
		struct {bool mode; bool rudder; bool HDM; bool CTS; bool deadband;} flag;

		//	Current Working Mode
		e_APmode mode;
		//	Current Rudder Position
		int16_t rudder;
		//	Heading Magnetic (HDM)
		whole_frac HDM;
		//	Course To Steer (CTS)
		whole_frac CTS;
		//	Deadband value
		whole_frac deadband;
		//	Trim
		//whole_frac trim;
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

	struct s_SOG {
			// Is Valid indicates if data is valid or not
			bool isValid=false;
			struct {bool SOG;} flag;

			whole_frac SOG;
	} ;

	struct s_boatSpeed {
		s_SOG SOG;
		long t0;
	} ;

// calibrate.py format
	struct s_calibrate_py {
		// Is Valid indicates if data is valid or not
		bool isValid=false;
		char sensor;
		struct {whole_frac x;whole_frac y;whole_frac z;} GAM_B;
		struct {whole_frac32 m11;whole_frac32 m12;whole_frac32 m13;
		whole_frac32 m21;whole_frac32 m22;whole_frac32 m23;
		whole_frac32 m31;whole_frac32 m32;whole_frac32 m33;} GAM_Ainv;


	};

	enum e_actions {
		NO_INSTRUCTION
		, START_STOP
		, INC_RUDDER_1
		, INC_RUDDER_10
		, DEC_RUDDER_1
		, DEC_RUDDER_10
		, INC_COURSE_1
		, INC_COURSE_10
		, INC_COURSE_100
		, DEC_COURSE_1
		, DEC_COURSE_10
		, DEC_COURSE_100
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
//		//Start compass calibration
//		, START_CAL
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
		, CAL_GYRO
		, CAL_ACCEL
		, CAL_MAGNET
		, CAL_ALL
		//Load calibration parameters
		, LOAD_calibrate_py // Load external calibration parameters
		// SOW received from RMC message
		, SOG
	};

	struct  {
		// ver 1: Fenix v0.1
		// ver 2: save CHECK value before IMU, InstParam and PID structures
		// ver 3: additional space for IMU ICM20948
		int ver=3; // TODO: Version of EEPROM structure is not saved
		long Flag=0;
		long IMU=500;//25; //Length 48 (bno055) 112 (ICM20948)
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
			#ifndef VIRTUAL_ACTUATOR
			0, // centerTiller
			512, //maxRudder
			#else
			VA_DELTACENTEROFRUDDER, // centerTiller
			VA_MRA, //maxRudder
			#endif
			AVDSPEED, //avdSpeed
			type_instSide::STARBOARD , //instSide
			5, //rudDamping
			{2,50}, //magVariation
			{0,0}, //headAlign
			20, //offCourseAlarm
			#ifndef VIRTUAL_ACTUATOR
			0, //minFeedback
			1024 // maxFeedback
			#else
			VA_MINFEEDBACK, //minFeedback
			VA_MAXFEEDBACK // maxFeedback
			#endif
	};

enum e_start_stop {CURRENT_HEADING, CURRENT_TARGET};

class Autopilot: public ActuatorManager, public BearingMonitor {

public:
	Autopilot(s_gain gain=HC_GAIN, int ControllerDirection=REVERSE, s_instParam ip= HC_INSTPARAM);
	virtual ~Autopilot();

	// FUNCTIONAL MODULE: WORKING MODES
	e_setup_status setup();
	e_working_status Compute();
	bool setCurrentMode(e_APmode = STAND_BY, char sensor = '-');

	e_APmode getCurrentMode() const {
		return _currentMode;
	}
	String getCurrentModeStr() const {
		return _status[_currentMode];
	}
	float getTargetBearing() const {
		return _targetBearing;
	}

	void setTargetBearing(float targetBearing);


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
	void Start_Cancel_AutotunePID(void);

	//TRACK MODE
	bool activateWPnext(void); // User push Next button
	void APBreceived(s_APB APB);

	//EXTERNAL COMPASS MODE
	void HDMreceived(s_HDM HDM);

	void SOGreceived(s_SOG SOG);

	//WIND MODE
	void VWRreceived(s_VWR VWR);
	int getWindDir(void);

	//OVERLOADED FUNCTIONS
	int changeRudder(int delta_rudder);
    void SetTunings(double, double,       // * While most users will set the tunings once in the
                    double);         	  //   constructor, this function gives the user the option
                                          //   of changing tunings during runtime for Adaptive control
    bool setHeadingDev(float headingDev = 0);
	void setDBConf (type_DBConfig status);
	type_DBConfig nextDBConf (void);

	//FUNCTIONAL MODULE: EEPROM
	void EEPROM_setup();
	void EEPROM_format();
	void EEsave_ReqCal (char sensor = '0');
	char EEload_ReqCal (void);
	bool EEsave_Calib();
	e_IMU_cal_status EEload_Calib();
	bool EEsave_HCParam(); //Save HARDCODED InstParam and PIDgain
	bool EEsave_instParam(bool HC=false);
	bool EEload_instParam();
	bool EEsave_PIDgain(bool HC=false);
	bool EEload_PIDgain ();
	bool isCalMode(void);
	bool isExternalCalibration(void);

	bool EEload_CHECK (long address);
	void EEsave_CHECK (long address);

	bool Load_calibrate_py (s_calibrate_py calibrate_py);

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
	void Start_Cal(char sensor = '-');
	void Cal_NextSensor(void);
	void Cancel_Cal();
	void Save_Cal(){
		EEsave_Calib();
		EEsave_ReqCal('0'); // Update Calibration Flag to disabled
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

	void setWarning(e_warning warning = NO_WARNING, bool instant = false) {

		if (_warning != warning) {

			//if there are Warnings pending to be displayed, these will be lost!
			if (_pending_W == true) _lost_W =true;

			_warning = warning;
			_pending_W = true;
		}
		if (instant) printWarning(instant);
	}

	void printWarning(bool instant = false);

	e_info getInformation() const {
		return _information;
	}


	void setInformation(e_info information = NO_MESSAGE) {
		_information = information;
		sprintf(DEBUG_buffer,"!INFORMATION Code: %i\n", _information);
		DEBUG_print();
		buzzer_Information();
	}

	void print_PIDFrontend (void);
private:
	e_APmode _currentMode= STAND_BY; // current working mode
	float _targetBearing= 0; // target vessel bearing
	float _nextCourse= 0; // Next course in STDBY/ AUTO Mode
	String _status[5] = { "STAND BY", "FOLLOW BEARING", "CALIBRATING" };
	char _sensor = '-';
	char* deblank(char* input)
	{
	    int i,j;
	    char *output=input;
	    for (i = 0, j = 0; i<strlen(input); i++,j++)
	    {
	        if (input[i]!=' ')
	            output[j]=input[i];
	        else
	            j--;
	    }
	    output[j]=0;
	    return output;
	}


	// FUNCTIONAL MODULE: WORKING MODES
	bool before_changeMode(e_APmode newMode, e_APmode currentMode, char sensor);
	bool after_changeMode(e_APmode currentMode, e_APmode preMode);
	e_working_status compute_OperationalMode(void);
	e_working_status compute_Stand_By(void);
	e_working_status compute_Cal_IMU(void);
	e_working_status compute_Cal_Feedback(void);
	e_working_status compute_Autotune(void);

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


	//Boat speed from RMC
	s_boatSpeed _boatSpeed;
	void set_boatSpeed(s_SOG SOG);
	bool isValid_boatSpeed (void);
	float get_boatSpeed (void);

	//Wind mode
	s_windDir _windDir;
	int _targetWindDir; // Target wind direction relative to heading
	void set_windDir(s_VWR VWR);
	bool isValid_VWR (void);


	void reset(){
		DEBUG_print(F("Reset\n"));
		delay (2000);
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
	bool _offCourseAlarmIDLE = false; // Alarm is not working until ship is within OCA angle since user changed target bearing.
	// This is to avoid alarm too soon when making high turns that takes its time (eg. tacking +100 degrees).
	double const _offCourseMaxTime = 15000; // max off course time before alarm starts
	bool _offCourseAlarmActive=false;
	bool compute_OCA (float delta);

	uint8_t getOffCourseAlarm() const {
		return _offCourseAlarm;
	}

	void setOffCourseAlarm(uint8_t offCourseAlarm) {
		_offCourseAlarm = offCourseAlarm;
	}


	// FUNCTIONAL MODULE: BOAT SPEED
	uint8_t _avgSpeed;

	uint8_t getAvgSpeed() const {
		return _avgSpeed;
	}
	void setAvgSpeed (uint8_t avgSpeed) {
		if (avgSpeed == 0) avgSpeed =1;
		if (avgSpeed > 10) avgSpeed =10;
		_avgSpeed = avgSpeed;
	}

	// FUNCTIONAL MODULE: ERROR, WARNING AND INFORMATION DISPLAY
	e_info _information = NO_MESSAGE;
	e_warning _warning = NO_WARNING;
	e_error _error = NO_ERROR;
	bool _pending_W = false;
	bool _lost_W = false;

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
