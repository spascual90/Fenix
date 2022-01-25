/*
 * MacuaAutopilot.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "Autopilot.h"

Autopilot::Autopilot( s_gain gain, int ControllerDirection, s_instParam ip)
 : ActuatorManager(gain.Kp.float_00(), gain.Ki.float_00(), gain.Kd.float_00(), ControllerDirection, ip.maxRudder, ip.rudDamping, ip.centerTiller, ip.minFeedback, ip.maxFeedback) //maxRudder is the min/max value for PID as well.
 , Bearing_Monitor ( ip.headAlign.float_00() )
{
	//SET HARDCODED INSTALATION PARAMETERS
	// Installation side IS
	setInstallationSide(ip.instSide); //TODO: implement STAR/PORTBOARD installation
	//	Magnetic variation DM
	setDm(ip.magVariation.float_00());
	// Active Working mode at start-up is Stand-by.
	_currentMode = STAND_BY; //DON'T USE SetCurrentMode in this constructor
	//Off course alarm angle OCA
	setOffCourseAlarm(ip.offcourseAlarm);
	//TODO:Average cruise speed ACS

}

Autopilot::~Autopilot() {
	// TODO Auto-generated destructor stub
}

e_setup_status Autopilot::setup() {


	// Autopilot version
	DEBUG_print("Fenix Autopilot: ");
	DEBUG_print(ARDUINO_VERSION);
	DEBUG_print("\n");

#ifdef RESTORE_EEPROM
	//Format EEPROM
	EEPROM_format();
#endif

	// Setup buzzer
	buzzer_setup();


	setInformation(SETUP_INPROGRESS);

	//Setup EEPROM
	EEPROM_setup();

	// Setup linear actuator
	ActuatorController::setup();

	// Setup Actuator manager
	ActuatorManager::setup();


	// Setup rudder feedback
	e_feedback_status f_status = RudderFeedback::setup(true);

	switch (f_status) {
	case ERROR_TOO_BIG:
		DEBUG_print("!WARNING: Check your linear actuator is powered-on and connected!\n");
		setWarning(FBK_ERROR_HIGH);
		//return FEEDBACK_ERROR; Not considered an error
		break;
	case OK_VIRTUAL:
		DEBUG_print("!Debugging: Virtual actuator enabled\n");
		break;
	case FEEDBACK_OK:
		break;
	}

	// Setup IMU

	switch (Bearing_Monitor::setup()) {
	case NOT_DETECTED:
		// There was a problem detecting the IMU ... check your connections */
		DEBUG_print("!WARNING: IMU Not detected. Check your wiring or I2C ADDR\n");
		setWarning(IMU_NOTFOUND);
		//return IMU_ERROR; Lack of IMU is not an error anymore. External IMU can be used instead
		return SETUP_OK;
		break;
	case SIMULATED:
		DEBUG_print("!Debugging: IMU simulator enabled\n");
		return SETUP_OK;
		break;
	default:
		break;
	}

	//Enter into calibration mode
	if (EEload_ReqCal()) {
		EEsave_ReqCal(false); // Set flag to disabled to avoid entering into Calibration mode each time
		DEBUG_print("!Calibration mode!\n");
		// Launch calibration
		setCurrentMode (CAL_IMU_COMPLETE);
		return SETUP_OK;
		//if (BNO_GetCal(true)==RECALIBRATED) { // Calibrates and checks results
		//	DEBUG_print("!Calibration succeed!\n");
		//	return SETUP_OK;
		//}
	  }

	 //  Get and restore BNO Calibration offsets
	 if (EEload_Calib()==CAL_RESULT_NOT_CALIBRATED) {
		// There was a problem reading IMU calibration values
		//EEsave_ReqCal(true); // Set flag to enabled to enter into Calibration mode next time system is reset
		DEBUG_print("!Please enter into Calibration Mode\n"); // System is not reset automatically to avoid recurrent writing EEPROM
		setWarning(EE_IMU_NOTFOUND);
		return SETUP_OK;
	 }
	// Ensures calibration is valid by recalibrating (without check feature)
	refreshCalStatus();
	reset_calibration();


	setInformation(NO_MESSAGE);
	return SETUP_OK;
}

e_working_status Autopilot::Compute() {
	e_working_status ws = RUNNING_OK;

#ifdef SHIP_SIM
	SIM_updateShip(getCurrentRudder());
#endif

	// Once each XXX loops: Update current course and target bearing (in track mode). Stores value for later use.
	computeLongLoop();


	// Play buzzer if required
	buzzer_play();

	// Updates current rudder just once each loop. Stores value for later use.
	if (updateCurrentRudder()<0) return RUNNING_ERROR;

	switch (_currentMode) {
	case AUTO_MODE:
	case TRACK_MODE:
	case WIND_MODE:
		ws = compute_OperationalMode();
		break;
	case STAND_BY:
		ws = compute_Stand_By();
		break;
	case CAL_IMU_COMPLETE:
		return RUNNING_OK;
		break;
	case CAL_FEEDBACK:
		ws = compute_Cal_Feedback();
		break;
	}

	// update IMU calibration except in CAL_IMU_COMPLETE
	if (_currentMode!=CAL_IMU_COMPLETE) ws = compute_Cal_IMU(false);


	return ws;
}

e_working_status Autopilot::compute_Cal_IMU(bool completeCal){
	if (!Bearing_Monitor::compute_Cal_IMU(completeCal)) {
		if (_currentMode==CAL_IMU_COMPLETE) setCurrentMode(STAND_BY);
	}

	return RUNNING_OK;
}

e_working_status Autopilot::compute_Cal_Feedback(){
	RudderFeedback::compute_Cal_Feedback();
	return RUNNING_OK;
}

e_working_status Autopilot::compute_Stand_By(){
	if (changeRudder(0)!=1) return RUNNING_ERROR;
	return RUNNING_OK;
}

e_working_status Autopilot::compute_OperationalMode(void){
	float PIDerrorPrima = delta180(getTargetBearing(), Bearing_Monitor::getCurrentHeading());
	if (PIDerrorPrima==-360) return RUNNING_ERROR;
	if (ActuatorManager::Compute(PIDerrorPrima)!=1) return RUNNING_ERROR;
	compute_OCA (PIDerrorPrima);
	return RUNNING_OK;
}


void Autopilot::computeLongLoop_TrackMode(void) {
	if (_currentMode == TRACK_MODE) {
		if (_WPactive.APB.isValid) {
			// Update course to steer
			setTargetBearing(_WPactive.APB.CTS.float_00());
		} else {
			// Cancel Track mode
			setInformation (NO_MESSAGE);
			setWarning (WP_INVALID);
			setCurrentMode(AUTO_MODE);
		}
	}
}

void Autopilot::computeLongLoop_WindDir(void) {

	// Evaluate validity of Wind info
	isValid_VWR ();

	// Act in consequence
	if (_currentMode == WIND_MODE) {
		if (_windDir.VWR.isValid) {
			// Update course to steer
			// CTS = HDG + VWR

			setTargetBearing(getCurrentHeading() + getWindDir() - _targetWindDir);
			setNextCourse(getTargetBearing());
			//sprintf(DEBUG_buffer,"!Target Bearing: %i\n", int(getTargetBearing()));
			//DEBUG_print();

		} else {
			// Cancel Wind mode
			setInformation (NO_MESSAGE);
			setWarning (NO_WIND_DATA);
			setCurrentMode(AUTO_MODE);
		}
	}
}


// RETURN true = Mode changed successfully
// false = Change mode aborted
bool Autopilot::setCurrentMode(e_APmode newMode) {

	bool rt = false;
	e_APmode prevMode = _currentMode;
	if (_currentMode == newMode) {return true;}

	//PRE-CHANGE MODE
	rt = before_changeMode(newMode, _currentMode);

	// Abort change mode
	if (!rt) {
		_currentMode = prevMode;
		return false;
	}

	//CHANGE MODE
	e_APmode preMode = _currentMode;
	_currentMode = newMode;

	switch (_currentMode) {
	case AUTO_MODE:
		//DEBUG_print("!setCurrentMode: AUTO_MODE\n");
		break;
	case WIND_MODE:
		//DEBUG_print("!setCurrentMode: WIND_MODE\n");
		break;
	case STAND_BY:
		//DEBUG_print("!setCurrentMode: STAND_BY\n");
		break;
	case CAL_IMU_COMPLETE:
		//Start IMU calibration
		//setInformation(IMU_CAL_INPROGRESS);
		break;

	case CAL_FEEDBACK:
		//set_calFeedback();
		start_calFeedback();
		break;
	default:
		break;
	}


	//POST-CHANGE MODE
	rt = after_changeMode(_currentMode, preMode);

	// Abort change mode
	if (!rt) {
		_currentMode = prevMode;
		return false;
	}
	return true;
}

void Autopilot::computeLongLoop() {
	// Once each XX loops: Update current course and target bearing (in track mode). Stores value for later use.
	if (IsLongLooptime ()) {

		refreshCalStatus();

		if (_currentMode == CAL_IMU_COMPLETE) compute_Cal_IMU(true);
		if (isCalMode()) return;

		computeLongLoop_heading();
		computeLongLoop_WP();
		computeLongLoop_TrackMode();
		computeLongLoop_WindDir();
		LongLoopReset();
	}

}


bool Autopilot::before_changeMode(e_APmode newMode, e_APmode currentMode){
	switch (currentMode) {
	case CAL_FEEDBACK:
		set_calFeedback();
		break;
	case TRACK_MODE:
		if (newMode == AUTO_MODE) {
			setTargetBearing (_WPactive.APB.CTS.float_00());
		}
		break;
	default:
		break;
	}
	return true;
}

bool Autopilot::after_changeMode(e_APmode currentMode, e_APmode preMode) {

	if (preMode == TRACK_MODE) {
		s_APB APB = {};
		setWPactive(APB);
		if (getWarning()==WP_INVALID) setWarning (NO_WARNING);
	}

	switch (currentMode) {
	case AUTO_MODE:
		// Same behavior as Track mode
	case TRACK_MODE:
		if (isHeadingValid()) {
			startAutoMode();
		} else {
			return false;
		}
		break;
	case STAND_BY:
		stopAutoMode();
		compute_OCA (0); //Stop off-course alarm if active
		break;
	default:
		break;
	}


	return true;
}

float Autopilot::getNextCourse() {
	if (_WPnext.APB.isValid) {
		return _WPnext.APB.CTS.float_00();
	} else {
		return _nextCourse;
	}
}

void Autopilot::setNextCourse(float nextCourse) {
	if (nextCourse<0) {nextCourse+= 360;}
	nextCourse = fmod (nextCourse, double(360));
	_nextCourse = nextCourse;
}

void Autopilot::setHeadingDev(float headingDev) {
	if (getCurrentMode() == STAND_BY) Bearing_Monitor::setHeadingDev(headingDev);
}

// CALIBRATION MODE
bool Autopilot::isCalMode(void){
	switch (getCurrentMode()) {
	case CAL_IMU_COMPLETE:
	case CAL_FEEDBACK:
		return true;
	default:
		return false;
	}

	return false;
}

void Autopilot::Start_Cal(){
	EEsave_ReqCal(true);// Update Calibration Flag to enabled
	reset();  //call reset
}

void Autopilot::Cancel_Cal(){
	reset();  //call reset
}


void Autopilot::Start_Stop(e_start_stop type){
	float target =-1;
	e_APmode mode = getCurrentMode();
	if (!isCalMode()) {

		switch (type) {
		case CURRENT_HEADING:
			target = getCurrentHeading();
			break;
		case CURRENT_TARGET:
			target = getNextCourse();
			break;
		}

		setNextCourse(target);
		setTargetBearing(target);

		switch (mode) {
		// If in STAND_BY --> set AUTO MODE
		case STAND_BY:
			setCurrentMode(AUTO_MODE);
			break;
		// If in TRACK MODE, WIND MODE or AUTO MODE --> set STAND_BY
		case TRACK_MODE:
		case AUTO_MODE:
		case WIND_MODE:
			setCurrentMode(STAND_BY);
			break;
		default:
			break;
		}

	}
}

void Autopilot::Start_Stop_wind(void){
	float target =-1;
	e_APmode mode = getCurrentMode();
	target = getCurrentHeading();
	setNextCourse(target);
	setTargetBearing(target);

	switch (mode) {
	// If in STAND_BY --> set AUTO MODE
	case WIND_MODE:
		setCurrentMode(AUTO_MODE);
		break;
	// If in AUTO MODE --> set WIND_MODE
	case AUTO_MODE:
		_targetWindDir= getWindDir();
		if (_targetWindDir!=-1) setCurrentMode(WIND_MODE);
		break;
	}
}


void Autopilot::Enter_Exit_FBK_Calib(void) {
	switch (getCurrentMode()) {
	// If in STAND_BY --> set FBK_CALIB mode
	case STAND_BY:
		setCurrentMode(CAL_FEEDBACK);
		break;
	case CAL_FEEDBACK:
		setCurrentMode(STAND_BY);
		break;
		// If other case REJECT
	default:
		break;
	}
}

// EXTERNAL COMPASS MODE
void Autopilot::set_extHeading(s_HDM HDM) {
	_extHeading.HDM = HDM;
	_extHeading.t0 = millis();
//	DEBUG_print ("!ext Heading received\n");
}

//evaluate validity extHeading
bool Autopilot::isValid_HDM (void) {
	//if ((_extHeading.HDM.isValid) &&
	//bool prev_isValid = _extHeading.HDM.isValid;
	_extHeading.HDM.isValid = (millis()-_extHeading.t0)<=MAX_HDM_TIME;
//	if (prev_isValid!=_extHeading.HDM.isValid){
//		if (prev_isValid == false) {
//			DEBUG_print ("!EXT Heading\n");
//			} else {
//			DEBUG_print ("!INT Heading\n");
//		}
//	}

	return _extHeading.HDM.isValid;
}

// WIND MODE
void Autopilot::set_windDir(s_VWR VWR) {
	_windDir.VWR = VWR;
	_windDir.t0 = millis();
	//DEBUG_print ("!wind received\n");
}

//evaluate validity windDir
bool Autopilot::isValid_VWR (void) {
	_windDir.VWR.isValid = (millis()-_windDir.t0)<=MAX_VWR_TIME;
	return _windDir.VWR.isValid;
}

// Return relative wind direction as an int angle between 0 and 359
//if not valid data available return -1
int Autopilot::getWindDir(void) {
	if (_windDir.VWR.isValid) return (_windDir.VWR.windDirLR=='L'?360-_windDir.VWR.windDirDeg.whole:_windDir.VWR.windDirDeg.whole);
	return -1;
}

// TRACK MODE
void Autopilot::setWPactive(s_APB APB) {
	_WPactive.APB = APB;
	_WPactive.t0 = millis();
	//DEBUG_print ("!WPactive valid\n");
}

void Autopilot::setWPnext(s_APB APB){
	_WPnext.APB = APB;
	_WPnext.t0 = millis();
	//DEBUG_print ("!WPnext valid\n");

}

bool Autopilot::activateWPnext(void) {
	// User push Next button
	if (_WPnext.APB.isValid) {
		setWPactive(_WPnext.APB);
		_WPnext.APB.isValid = false;
		setNextCourse(_WPnext.APB.CTS.float_00());

		return true;
	}
	return false;
}

void Autopilot::APBreceived(s_APB APB) {

	if (_WPactive.APB.isValid) {
		if (strcmp(_WPactive.APB.destID, APB.destID)==0) {
			setWPactive(APB);
			setInformation (TRACKING);
		} else {
			setWPnext(APB);
			setInformation (CONFIRM_NEW_WP);
		}
	} else {
		setWPnext(APB);
		setInformation (TRACKMODE_AVAILABLE);
	}
}
//$ECAPB,A,A,0.00,L,N,V,V,312.23,M,001,312.34,M,312.34,M*2A

void Autopilot::HDMreceived(s_HDM HDM) {
	set_extHeading(HDM);
}

void Autopilot::VWRreceived(s_VWR VWR) {
	set_windDir(VWR);
}

void Autopilot::computeLongLoop_heading(void) {
	//if HDM messages are received within MAX_HDM_TIME seconds, external compass is valid.
	//only changes compass source (internal/ external) in STAND_BY
	updateHeading(_currentMode == STAND_BY, isValid_HDM(), _extHeading.HDM.HDM.float_00());

	//If heading value is not valid in operational mode set STAND BY mode
	if (!isHeadingValid() and !isCalMode() and getCurrentMode()!=STAND_BY) {
		setWarning(IMU_LOW);
		setCurrentMode(STAND_BY);
	}
}

void Autopilot::computeLongLoop_WP(void) {
	//evaluate validity WPactive
	if ((_WPactive.APB.isValid) && ((millis()-_WPactive.t0)>MAX_APB_TIME)) {
			_WPactive.APB.isValid = false;
			_WPactive.APB.destID[0]= '-';
			_WPactive.APB.destID[1]= '-';
			_WPactive.APB.destID[2]= '-';
			_WPactive.APB.destID[3]= '-';
			_WPactive.APB.destID[4]= '-';
			_WPactive.APB.destID[5]= '\n';

	}

	if ((_WPnext.APB.isValid) && ((millis()-_WPnext.t0)>MAX_APB_TIME)) {
			_WPnext.APB.isValid = false;
			_WPnext.APB.destID[0]= '-';
			_WPnext.APB.destID[1]= '-';
			_WPnext.APB.destID[2]= '-';
			_WPnext.APB.destID[3]= '-';
			_WPnext.APB.destID[4]= '-';
			_WPnext.APB.destID[5]= '\n';
			if (getInformation()==TRACKMODE_AVAILABLE) setInformation(NO_MESSAGE);
	}


}


// OVERLOADED FUNCTIONS
void Autopilot::SetTunings(double Kp, double Ki, double Kd) {
	if (!isCalMode()) ActuatorManager::PID_ext::SetTunings(Kp, Ki, Kd);
}

int Autopilot::changeRudder(int delta_rudder) {
	int ret =0;
	e_dir dir;
	switch (getCurrentMode()) {
	case CAL_FEEDBACK:
		dir=(delta_rudder>0?EXTEND:RETRACT);
		ret = cal_FBK_move(dir);
		break;
	case CAL_IMU_COMPLETE:
		break;
	default:
		setCurrentMode(STAND_BY);
		ret = ActuatorManager::changeRudder(delta_rudder);
	break;
	}

	return ret;
}

void Autopilot::setDBConf (type_DBConfig status) {
	if (!isCalMode()) dbt.setDBConf (status);
}




void Autopilot::Request_instParam(s_instParam & instParam) {
	instParam.centerTiller=getDeltaCenterOfRudder();
	instParam.maxRudder=getMaxRudder();
	//instParam.avgSpeed=MyPilot->; TODO: ImplementavdSpeed
	instParam.instSide=getInstallationSide();
	instParam.rudDamping=getErrorFeedback();
	instParam.magVariation.Towf_00(getDm());
	instParam.headAlign.Towf_00(getHeadingDev());
	instParam.minFeedback=getMinFeedback();
	instParam.maxFeedback=getMaxFeedback();
	//instParam.offcourseAlarm=MyPilot-;; TODO: Implement off course alarm
	instParam.flag = {true, true, true, true, true, true, true, true, true, true};
	instParam.isValid = true;
}

void Autopilot::buzzer_tone_start (unsigned long frequency, int duration) {
#ifdef BUZZER
	_buzzFrec = frequency;
	_buzzDur = duration;
	BuzzReset();
#endif
}

bool Autopilot::Change_instParam (s_instParam instParam) {
	bool rt = false;
	if (getCurrentMode() == STAND_BY) { //ONLY ALLOWED IN STAND_BY MODE
		if (instParam.flag.centerTiller) setDeltaCenterOfRudder(instParam.centerTiller);
		//TODO: change MRA
		//TODO: change average cruise speed
		//TODO: change installation side
		if (instParam.flag.rudDamping) setErrorFeedback(instParam.rudDamping);
		if (instParam.flag.magVariation) setDm(instParam.magVariation.float_00());
		if (instParam.flag.headAlign) setHeadingDev(instParam.headAlign.float_00());
		if (instParam.flag.minFeedback and instParam.flag.maxFeedback) {
			setMinFeedback(instParam.minFeedback, false);
			setMaxFeedback(instParam.maxFeedback, true);
		}
		//TODO: change off course alarm
		rt= true;
	}
	return rt;
}

void Autopilot::Request_PIDgain(s_PIDgain & PIDgain) {
	PIDgain.gain.Kp.Towf_00(float(GetKp()));
	PIDgain.gain.Ki.Towf_00(float(GetKi()));
	PIDgain.gain.Kd.Towf_00(float(GetKd()));
	PIDgain.sTime= GetSampleTime();
	PIDgain.DBConfig = dbt.getDBConf();
	PIDgain.flag = {{true, true, true}, true, true};
	PIDgain.isValid = true;

}



// BUZZER FUNCTIONAL MODULE
void Autopilot::buzzer_setup() {
#ifdef BUZZER
	//Setup Buzzer
	pinMode(PIN_BUZZER, OUTPUT); // Set buzzer - pin PIN_BUZZER as an output
	buzzer_IBIT();
#else
	DEBUG_print("!Debugging: SAFETY NOTICE: Buzzer disconnected!\n");
#endif
}

// Initial buzzer test
void Autopilot::buzzer_IBIT() {
	sprintf(DEBUG_buffer,"Buzzer test on PIN %i ...\n", get_PIN_BUZZER());
	DEBUG_print();

	// Performs an initial test of the buzzer
	tone(PIN_BUZZER, 1000, 1000); // Send 1KHz sound signal...
}

// Error: buzzer sound
void Autopilot::buzzer_Error() {
	buzzer_tone_start (2000, 40);
}

void Autopilot::buzzer_Warning() {
	buzzer_tone_start (2000, 20);
}

void Autopilot::buzzer_Information() {
	buzzer_tone_start (1000, 20);
}

void Autopilot::buzzer_Beep() {
	buzzer_tone_start (1000, 1);
}



// frequency -->Frequency of the sound
// duration--> Number of periods playing sound. Maximum 1023 (250 seg aprox)

void Autopilot::buzzer_noTone() {
	noTone(PIN_BUZZER);
	_Buzz=false; // Stop buzzer
}

void Autopilot::buzzer_play() {
	if (_Buzz) { // If buzzer is on
		tone(PIN_BUZZER, _buzzFrec); // Send sound signal

		if (IsBuzzTime()) { //End of period?
			BuzzReset();
			if ((_buzzDur--)<0) buzzer_noTone(); //End of sound condition?
		}
	}
}

void Autopilot::BuzzReset() {
	_Buzz=true;
	_DelayBuzzStart = millis();
}

bool Autopilot::IsBuzzTime () {
	// returns false if timer is ON and still RUNNING
	// returns true if timer is OFF or is ON but arrived to the limit TIME
	if ( !_Buzz or ( (millis() -_DelayBuzzStart) < DELAY_BUZZBEAT_TIME) ) {
		return false;}
	return true;
}


// OUT OF COURSE ALARM FUNCTIONAL MODULE
//arguments: delta - angle (-180, 179) to be compared against off course alarm angle.
// return true if out course more than x secs
// return false if in course
bool Autopilot::compute_OCA (float delta) {
	static double sd_offCourseStartTime;
	static bool sb_offCourse =false;
	bool l_offCourse;

	//detect if we are in-course-->stop alarm (if active)
	if (abs(delta) < _offCourseAlarm) {
		if (sb_offCourse ==true) {
			// reset static values
			sb_offCourse=false; // in course, stop counting
			_offCourseAlarmActive = false;
			buzzer_noTone(); // shut down alarm
			setWarning();
		}
		return _offCourseAlarmActive;
	} else l_offCourse = true;// else-->we are out of course

	if (l_offCourse == true and sb_offCourse == false) {// change detected-->start counting
		sb_offCourse = true;
		sd_offCourseStartTime = millis();
	}

	if (_offCourseAlarmActive == false and (millis()-sd_offCourseStartTime)>_offCourseMaxTime) {
		_offCourseAlarmActive = true;
		setWarning(OUT_OF_COURSE);
		buzzer_tone_start (1000, 1023);


	}

	return _offCourseAlarmActive;
}


//EEPROM FUNCTIONAL MODULE
void Autopilot::EEPROM_setup() {
	sprintf(DEBUG_buffer,"EEPROM V%i\n", EE_address.ver);
	DEBUG_print();
	if (!EEload_instParam()) {
		DEBUG_print("!WARNING: Could not load Installation parameters: Restoring default.\n");
		setWarning (EE_INSTPARAM_NOTFOUND);
		//Restore HARDCODED parameters but don't save!
		Change_instParam (HC_INSTPARAM);
	}

	if (!EEload_PIDgain()) {
		DEBUG_print("!WARNING: Could not load PID parameters: Restoring default.\n");
		setInformation (EE_PID_DEFAULT);
		//Restore HARDCODED parameters but don't save!
		SetTunings(HC_GAIN.Kp.float_00(), HC_GAIN.Ki.float_00(), HC_GAIN.Kd.float_00());
	}
}

void Autopilot::EEPROM_format() {
	DEBUG_print("!EEPROM Format...");

	for (uint16_t i = 0 ; i < EEPROM.length() ; i++) {
	    EEPROM.update(i, 0xFF);
	  }

	DEBUG_print("Ok\nStop.");
    while (1) {;}

}


void Autopilot::EEsave_ReqCal (bool reqCalib)
{
	const uint8_t TRUEvalue = CHECKvalue; // 170 = 10101010 in binary
	const uint8_t FALSEvalue = 0; // 0 = 00000000 in binary

	uint8_t value =0;
	value = reqCalib?TRUEvalue:FALSEvalue;
	EEPROM.put(EE_address.Flag, value); // true enables Calibration Flag to force calibration
}

void Autopilot::EEsave_CHECK (long address)
{
	EEPROM.put(address, CHECKvalue); // true enables Calibration Flag to force calibration
}

bool Autopilot::EEload_CHECK (long address)
{
	uint8_t value = 0;
	EEPROM.get(address, value); // true Require Calibration Flag to force calibration
	return (value == CHECKvalue?true:false);
}


bool Autopilot::EEload_ReqCal (void)
{
	uint8_t value = 0;

	EEPROM.get(EE_address.Flag, value); // true Require Calibration Flag to force calibration
	sprintf(DEBUG_buffer,"ReqCal: %i\n",value);
	DEBUG_print(DEBUG_buffer);
	if (value == CHECKvalue) return true;
	return false;
}

bool Autopilot::EEsave_Calib(){
	bool DataStored = false;
	long eeAddress = EE_address.IMU;
 	//DATA TO SAVE
	long bnoID;
	adafruit_bno055_offsets_t Calib;

	DEBUG_print("!Saving Calibration...");

	//bnoID
    bnoID = getSensorId();
    EEPROM.put(eeAddress, bnoID);
    eeAddress += sizeof(bnoID);

    //Calib
	Calib = getSensorOffsets();
    EEPROM.put(eeAddress, Calib);

    DataStored = true;

	DEBUG_print("Ok\n");
    return DataStored;
}  //  end EEsave_Calib

e_IMU_cal_status Autopilot::EEload_Calib(){
	//  Get and restore BNO Calibration offsets
	long EE_bnoID, bnoID;
	adafruit_bno055_offsets_t calibrationData;
	int eeAddress = EE_address.IMU;

	//  Look for the sensor's unique ID in EEPROM.
	EEPROM.get(eeAddress, EE_bnoID);
	// Look for unique ID reported by IMU
	bnoID = getSensorId();
	sprintf(DEBUG_buffer,"IMU Sensor ID: %i\n",bnoID);
	DEBUG_print(DEBUG_buffer);

	if (EE_bnoID != bnoID) {
		DEBUG_print("!WARNING: No Calibration Data for this sensor found!\n");
		sprintf(DEBUG_buffer,"ID found: %i\n",EE_bnoID);
		DEBUG_print(DEBUG_buffer);

		return CAL_RESULT_NOT_CALIBRATED;
	}
	DEBUG_print("!Found Calibration data...\n");
	eeAddress += sizeof(long);
	EEPROM.get(eeAddress, calibrationData);
	setIniCalib(calibrationData);
	displaySensorOffsets();

	return CAL_RESULT_RECALIBRATED;
}


bool Autopilot::EEsave_HCParam(){
	bool inst_OK, PID_OK;
	inst_OK = EEsave_instParam(true);
	PID_OK = EEsave_PIDgain(true);
	return (inst_OK && PID_OK);
}

bool Autopilot::EEsave_instParam(bool HC){
	bool DataStored=false;
	//DATA TO SAVE
	s_instParam instParam;
	int eeAddress = EE_address.InstParam;
	DEBUG_print("!Saving InstParam...");

    //instParam_CHECK
    EEPROM.put(eeAddress, CHECKvalue);
    eeAddress += sizeof(CHECKvalue);

    // InstParam
	if (HC==true) {
		DEBUG_print("!Restored manufacturer values.");
		instParam=HC_INSTPARAM;}
	else {
		Request_instParam(instParam);
	}

    EEPROM.put(eeAddress, instParam);

    DataStored = true;
	DEBUG_print("Ok\n");
    return DataStored;
}

bool Autopilot::EEload_instParam (void){
	bool Loaded = false;
	long eeAddress = EE_address.InstParam;
	uint8_t check;
	s_instParam instParam;

    //PID_CHECK
	EEPROM.get(eeAddress, check);
    eeAddress += sizeof(check);

    if (check == CHECKvalue) {
		EEPROM.get(eeAddress, instParam);
		Loaded = instParam.isValid &&
				instParam.flag.avgSpeed && instParam.flag.centerTiller &&
				instParam.flag.headAlign && instParam.flag.instSide && instParam.flag.magVariation &&
				instParam.flag.maxRudder && instParam.flag.offcourseAlarm && instParam.flag.rudDamping &&
				instParam.minFeedback && instParam.maxFeedback;
		if (Loaded) Change_instParam (instParam);
		DEBUG_print();

    }
	return Loaded;
}


bool Autopilot::EEsave_PIDgain(bool HC){
	bool DataStored=false;
	long eeAddress = EE_address.PIDgain;
	s_PIDgain PIDgain;
	DEBUG_print("!Saving PIDgain...");

    //PID_CHECK
    EEPROM.put(eeAddress, CHECKvalue);
    eeAddress += sizeof(CHECKvalue);

	if (HC==true) {
		PIDgain = HC_PIDGAIN;
		DEBUG_print("Restored manufacturer values.");
	} else {
		Request_PIDgain(PIDgain);
	}
    EEPROM.put(eeAddress, PIDgain);

    //BORRAME
    sprintf(DEBUG_buffer,"pid Saved at %i\n", eeAddress );
	DEBUG_print();
	bool Loaded = PIDgain.isValid &&
			PIDgain.flag.DBConfig &&
			PIDgain.flag.sTime &&
			PIDgain.flag.gain.Kp && PIDgain.flag.gain.Ki && PIDgain.flag.gain.Kd;
    if (!Loaded) DEBUG_print("!Flags not true\n" );
    //BORRAME

    DataStored = true;
	DEBUG_print("!Ok\n");
    return DataStored;
}

bool Autopilot::EEload_PIDgain (void){
	bool Loaded = false;
	long eeAddress = EE_address.PIDgain;
	uint8_t check;
	s_PIDgain PIDgain;

    //PID_CHECK
	EEPROM.get(eeAddress, check);
    eeAddress += sizeof(check);

    if (check == CHECKvalue) {
		//PIDgain
		EEPROM.get(eeAddress, PIDgain);
		Loaded = PIDgain.isValid &&
				PIDgain.flag.DBConfig &&
				PIDgain.flag.sTime &&
				PIDgain.flag.gain.Kp && PIDgain.flag.gain.Ki && PIDgain.flag.gain.Kd;
    }

	if (Loaded) {
		SetTunings(PIDgain.gain.Kp.float_00(), PIDgain.gain.Ki.float_00(), PIDgain.gain.Kd.float_00());
		setDBConf(PIDgain.DBConfig);
	}

	return Loaded;
}

////CRC-8 - based on the CRC8 formulas by Dallas/Maxim
////code released under the therms of the GNU GPL 3.0 license
//byte Autopilot::CRC8(const byte *data, size_t dataLength)
//{
//  byte crc = 0x00;
//  while (dataLength--)
//  {
//    byte extract = *data++;
//    for (byte tempI = 8; tempI; tempI--)
//   {
//      byte sum = (crc ^ extract) & 0x01;
//      crc >>= 1;
//      if (sum)
//     {
//        crc ^= 0x8C;
//      }
//      extract >>= 1;
//    }
//  }
//  return crc;
//}
