/*
 * MacuaAutopilot.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
*/

#include "Autopilot.h"
#include <MemoryFree.h>
#include <simplot.h> //SIMPLOT FOR DEBUGGING PURPOSE ONLY

Autopilot::Autopilot( s_gain gain, int ControllerDirection, s_instParam ip)
 : ActuatorManager(gain.Kp.float_00(), gain.Ki.float_00(), gain.Kd.float_00(), ControllerDirection, ip.maxRudder, ip.rudDamping, ip.centerTiller, ip.minFeedback, ip.maxFeedback, float(ip.avgSpeed) ) //maxRudder is the min/max value for PID as well.
 , BearingMonitor ( ip.headAlign.float_00() )
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
	//Average cruise speed ACS
	setAvgSpeed (ip.avgSpeed);

}

Autopilot::~Autopilot() {
	// TODO Auto-generated destructor stub
}

e_setup_status Autopilot::setup() {

	sprintf(DEBUG_buffer,"!Free memory: %i/8192 bytes\n", freeMemory());
	DEBUG_print();

	// Autopilot version
	DEBUG_print(F("Fenix Autopilot: "));
	DEBUG_print(ARDUINO_VERSION);
	DEBUG_print(F("\n"));

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
	ActuatorManager::setup( ATUNE_NOISE, ATUNE_STEP, ATUNE_LOOKBACK );

	// Setup rudder feedback
	e_feedback_status f_status = RudderFeedback::setup(true);

	switch (f_status) {
	case ERROR_TOO_BIG:
		DEBUG_print(F("!W: L.Actuator error too big!\n")); //Check your linear actuator is powered-on and connected
		setWarning(FBK_ERROR_HIGH, true);
		//return FEEDBACK_ERROR; Not considered an error
		break;
	case OK_VIRTUAL:
		DEBUG_print(F("!Debug:Virt.actuator\n"));
		break;
	case FEEDBACK_OK:
		break;
	}

	// For debuging purpose only
	//reset_calibration(EE_address.IMU);

	// Setup IMU
	switch (IMU_setup(EE_address.IMU)) {
	case NOT_DETECTED:
		// There was a problem detecting the IMU ... check your connections */
		DEBUG_print(F("!W: IMU Not detected\n")); // Check your wiring or I2C ADDR
		setWarning(IMU_NOTFOUND, true);
		//return IMU_ERROR; Lack of IMU is not an error anymore. External IMU can be used instead
		return SETUP_OK;
		break;
	case SIMULATED:
		DEBUG_print(F("!Debug: IMU simulator\n"));
		return SETUP_OK;
		break;
	default:
		break;
	}

	//Enter into calibration mode
	char sensor = EEload_ReqCal();
	if (sensor == 'G' or sensor == 'A' or sensor == 'M' or sensor == '-') {
		EEsave_ReqCal('0'); // Set flag to disabled to avoid entering into Calibration mode each time
		DEBUG_print(F("!Calibration mode!\n"));
		// Launch calibration
		setCurrentMode (CAL_IMU_COMPLETE, sensor);
		return SETUP_OK;
	}

	//  Get and restore IMU Calibration offsets
	if (EEload_Calib()==CAL_RESULT_NOT_CALIBRATED) {
		// There was a problem reading IMU calibration values
		DEBUG_print(F("!Enter Calibration Mode\n")); // System is not reset automatically to avoid recurrent writing EEPROM
		setWarning(EE_IMU_NOTFOUND, true);
		return SETUP_OK;
	}

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

	// Display new warnings
	printWarning();

	// Play buzzer if required
	buzzer_play();

	#ifdef VIRTUAL_ACTUATOR
	ActuatorManager::compute_VA();
	#endif
	// Updates current rudder just once each loop. Stores value for later use.
	if (updateCurrentRudder()<0) return RUNNING_ERROR;

	switch (_currentMode) {
	case AUTO_MODE:
	case TRACK_MODE:
	case WIND_MODE:
		ws = compute_OperationalMode();

		//plot2(NeoSerial,int(getInput()), -int(getKdContrib()));//,int(getOutput(), int(getKpContrib()), int(getITerm()))
//		getTargetBearing();
//		getInput();
//		getCurrentRudder();
//		float (getKpContrib());
//		float (getITerm());
//		float(getKdContrib());
//		float(getOutput());
//		//_V[AI_DELTA_CRUDDER] = MyPilot->getDeltaCenterOfRudder();
//		//dbt.getDeadband();
//		dbt.getTrim();

		//



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
	case CAL_AUTOTUNE:
		ws = compute_Autotune();
		break;
	}

	return ws;
}

e_working_status Autopilot::compute_Cal_IMU(void){

	if (!BearingMonitor::compute_Cal_IMU(_sensor)) {
		if (_currentMode==CAL_IMU_COMPLETE) setCurrentMode(STAND_BY);
	}

	return RUNNING_OK;
}

e_working_status Autopilot::compute_Cal_Feedback(){
	RudderFeedback::compute_Cal_Feedback();
	return RUNNING_OK;
}

e_working_status Autopilot::compute_Stand_By(){
	//if (changeRudder(0)!=1) return RUNNING_ERROR;
	changeRudder(0);
	return RUNNING_OK;
}

e_working_status Autopilot::compute_OperationalMode(void){
	float PIDerrorPrima = delta180(getTargetBearing(), BearingMonitor::getCurrentHeading());
	if (PIDerrorPrima==-360) return RUNNING_ERROR;
	// Adapt to actual SOG or SOW speed over water
	if (ActuatorManager::Compute(PIDerrorPrima, get_boatSpeed())!=1) return RUNNING_ERROR;
	compute_OCA (PIDerrorPrima);
	return RUNNING_OK;
}

e_working_status Autopilot::compute_Autotune(void){
	float PIDerrorPrima = delta180(getTargetBearing(), BearingMonitor::getCurrentHeading());
	if (PIDerrorPrima==-360) return RUNNING_ERROR;
	if (ActuatorManager::Compute_Autotune(PIDerrorPrima)!=1) return RUNNING_ERROR;
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
bool Autopilot::setCurrentMode(e_APmode newMode, char sensor) {

	bool rt = false;
	e_APmode prevMode = _currentMode;
	if (_currentMode == newMode) {return true;}

	//PRE-CHANGE MODE
	rt = before_changeMode(newMode, _currentMode, sensor);

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
		//DEBUG_print(F("!setCurrentMode: AUTO_MODE\n"));
		break;
	case WIND_MODE:
		//DEBUG_print(F("!setCurrentMode: WIND_MODE\n"));
		break;
	case STAND_BY:
		//DEBUG_print(F("!setCurrentMode: STAND_BY\n"));
		break;
	case CAL_IMU_COMPLETE:
		//Start IMU calibration
		//setInformation(IMU_CAL_INPROGRESS);
		break;

	case CAL_FEEDBACK:
		//set_calFeedback();
		start_calFeedback();
		break;

	case CAL_AUTOTUNE:
		startAutoTune();
    	DEBUG_print(F("!SetCurrentMode: CAL_AUTOTUNE\n"));
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

#ifndef SHIP_SIM
		refreshCalStatus();
#endif
		if (_currentMode == CAL_IMU_COMPLETE) compute_Cal_IMU();
		if (_currentMode == CAL_AUTOTUNE) computeLongLoop_heading();

		if (isCalMode()) return;

		computeLongLoop_heading();

		// Once each XX loops: Update target bearing (in track mode). Stores value for later use.
		if (IsLongLooptime ()) {
			computeLongLoop_WP();
			computeLongLoop_TrackMode();
			computeLongLoop_WindDir();


			LongLoopReset();
		}
}


bool Autopilot::before_changeMode(e_APmode newMode, e_APmode currentMode, char sensor){
	switch (currentMode) {
	case CAL_FEEDBACK:
		set_calFeedback();
		break;
	case TRACK_MODE:
		if (newMode == AUTO_MODE) {
			setTargetBearing (_WPactive.APB.CTS.float_00());
		}
		break;

	case STAND_BY:
		if (newMode == CAL_AUTOTUNE) {
			setTargetBearing (getCurrentHeading());
		}

		if (newMode == CAL_IMU_COMPLETE) {
			_sensor = sensor;
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

	_offCourseAlarmIDLE = false; // OCA Alarm deactivated until ship heading gets into OCA angle
	DEBUG_print(F("OCA Alarm: Deactivated\n"));

	if (preMode == CAL_IMU_COMPLETE) {
		if (this->isExternalCalibration()) {
			// Reset autopilot after external calibration (eg. ICM_20948)
			reset();
		} else {
			// Print new calibration values after internal calibration
			BearingMonitor::displaySensorOffsets();
		}
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

void Autopilot::setTargetBearing(float targetBearing) {
		if (targetBearing<0) {targetBearing+= 360;}
		// If next course represents a big change in course...
		float delta_tb = abs (delta180(_targetBearing, targetBearing));
		// ...then OCA Alarm is deactivated until ship heading gets into OCA angle
		if (delta_tb > getOffCourseAlarm()) {
			_offCourseAlarmIDLE = false;
			DEBUG_print(F("OCA Alarm: Deactivated\n"));
		}
		// ...then Integral Term of PID is reset
		//ActuatorManager::PID_ext::resetITerm(delta_tb);
		_targetBearing = fmod (targetBearing, double(360));
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

bool Autopilot::setHeadingDev(float headingDev) {
	// Heading Deviation is only applicable to internal IMU. When receiving external IMU information this function is not operative
	if (getCurrentMode() == STAND_BY and  getIMUstatus()!=EXTERNAL_IMU ) {
		BearingMonitor::setHeadingDev(headingDev);
	} else {
		return false;
	}
	return true;
}

// CALIBRATION MODE
bool Autopilot::isCalMode(void){
	switch (getCurrentMode()) {
	case CAL_IMU_COMPLETE:
	case CAL_FEEDBACK:
	case CAL_AUTOTUNE:
		return true;
	default:
		return false;
	}

	return false;
}

void Autopilot::Start_Cal(char sensor){
	EEsave_ReqCal(sensor);// Update Calibration Flag to enabled
	reset();  //call reset
}

void Autopilot::Cal_NextSensor(void){
	BearingMonitor::Cal_NextSensor();
}

bool Autopilot::isExternalCalibration(void){
	return BearingMonitor::isExternalCalibration();
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


void Autopilot::Start_Cancel_AutotunePID(void) {
	switch (getCurrentMode()) {
	// If in STAND_BY --> set AutotunePID mode
	case STAND_BY:
		setCurrentMode(CAL_AUTOTUNE);
		break;
	case CAL_AUTOTUNE:
		setCurrentMode(STAND_BY);
		break;
		// In other case REJECT
	default:
		break;
	}
}

// EXTERNAL COMPASS MODE
void Autopilot::set_extHeading(s_HDM HDM) {
	_extHeading.HDM = HDM;
	_extHeading.t0 = millis();
//	DEBUG_print ("!ext Heading received\n"));
}

//evaluate validity extHeading
bool Autopilot::isValid_HDM (void) {
	_extHeading.HDM.isValid = (millis()-_extHeading.t0)<=MAX_HDM_TIME;
	//reset value of extHeading if last received is not valid any more
	if (!_extHeading.HDM.isValid) _extHeading.t0=0;
//	if (prev_isValid!=_extHeading.HDM.isValid){
//		if (prev_isValid == false) {
//			DEBUG_print ("!EXT Heading\n"));
//			} else {
//			DEBUG_print ("!INT Heading\n"));
//		}
//	}

	return _extHeading.HDM.isValid;
}

// EXTERNAL SPEED RECEIVED
void Autopilot::set_boatSpeed(s_SOG SOG) {
	_boatSpeed.SOG = SOG;
	_boatSpeed.t0 = millis();
	//DEBUG_print ("!ext Speed received\n");
}

float Autopilot::get_boatSpeed (void) {

	if (isValid_boatSpeed()) {
		return _boatSpeed.SOG.SOG.float_00();
	} else {
		return float (getAvgSpeed());
	}
}

//evaluate validity boatSpeed
bool Autopilot::isValid_boatSpeed (void) {
	_boatSpeed.SOG.isValid = (millis()-_boatSpeed.t0)<=MAX_SOG_TIME;
	//reset value of _boatSpeed if last received is not valid any more
	if (!_boatSpeed.SOG.isValid) {
		_boatSpeed.t0=0;
	}
	return _boatSpeed.SOG.isValid;
}

// WIND MODE
void Autopilot::set_windDir(s_VWR VWR) {
	_windDir.VWR = VWR;
	_windDir.t0 = millis();
	//DEBUG_print ("!wind received\n"));
}

//evaluate validity windDir
bool Autopilot::isValid_VWR (void) {
	_windDir.VWR.isValid = (millis()-_windDir.t0)<=MAX_VWR_TIME;
	// Last direction/speed remains. Dont reset.
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
	//DEBUG_print ("!WPactive valid\n"));
}

void Autopilot::setWPnext(s_APB APB){
	_WPnext.APB = APB;
	_WPnext.t0 = millis();
	//DEBUG_print ("!WPnext valid\n"));

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
//	sprintf(DEBUG_buffer,"CTS %i.%i\n", APB.CTS.whole, APB.CTS.frac );
//	DEBUG_print();
}

void Autopilot::HDMreceived(s_HDM HDM) {
	set_extHeading(HDM);
}

void Autopilot::VWRreceived(s_VWR VWR) {
	set_windDir(VWR);
}

void Autopilot::SOGreceived(s_SOG SOG) {
	set_boatSpeed (SOG);
}

void Autopilot::computeLongLoop_heading(void) {
	//if HDM messages are received within MAX_HDM_TIME seconds, external compass is valid.
	//only changes compass source (internal/ external) in STAND_BY

	BearingMonitor::updateHeading(_currentMode == STAND_BY, isValid_HDM(), _extHeading.HDM.HDM.float_00(), _extHeading.t0);

	if (isHeadingFrozen()) setWarning(IMU_LOW);

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
	if (!isCalMode()) setTypeDB (status);
}

type_DBConfig Autopilot::nextDBConf (void) {
	if (!isCalMode()) {
		type_DBConfig DBtemp;
		DBtemp = getTypeDB();
		//Loops through type_DBConfig values
		DBtemp = static_cast<type_DBConfig>((DBtemp + 1) % type_DBConfig::LOOP);
		setTypeDB(DBtemp);
		return DBtemp;
	}
	return getTypeDB();
}


void Autopilot::Request_instParam(s_instParam & instParam) {
	instParam.centerTiller=getDeltaCenterOfRudder();
	instParam.maxRudder=getMaxRudder();
	instParam.avgSpeed=getAvgSpeed();
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
	PIDgain.gain.Kp.Towf_00(float(PID::GetKp()));
	PIDgain.gain.Ki.Towf_00(float(PID::GetKi()));
	PIDgain.gain.Kd.Towf_00(float(PID::GetKd()));
	PIDgain.sTime= GetSampleTime();
	PIDgain.DBConfig = getTypeDB();//getDBConf();
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
	DEBUG_print(F("!Debugging: SAFETY NOTICE: Buzzer disconnected!\n"));
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

	//Detect if we are in-course-->stop alarm (if active)
	if (abs(delta) < _offCourseAlarm) {
		if (!_offCourseAlarmIDLE) {
			_offCourseAlarmIDLE = true; // Alarm in Stand By
			DEBUG_print(F("OCA Alarm: Stand by\n"));
		}
		if (sb_offCourse ==true) {
			// reset static values
			sb_offCourse=false; // in course, stop counting
			_offCourseAlarmActive = false;
			buzzer_noTone(); // shut down alarm
			setWarning();
		}
		return _offCourseAlarmActive;

	} else l_offCourse = true;// else-->we are out of course

	// change detected and alarm in Stand by-->start counting
	if (_offCourseAlarmIDLE == true and l_offCourse == true and sb_offCourse == false) {
		sb_offCourse = true;
		sd_offCourseStartTime = millis();
	}

	if (_offCourseAlarmIDLE == true and
		_offCourseAlarmActive == false and
		(millis()-sd_offCourseStartTime)>_offCourseMaxTime) {

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
		DEBUG_print(F("!W:Could not load Inst.Param"));
		DEBUG_print(F(". Restoring default.\n"));
		setWarning (EE_INSTPARAM_NOTFOUND, true);
		//Restore HARDCODED parameters but don't save!
		Change_instParam (HC_INSTPARAM);
	}

	if (!EEload_PIDgain()) {
		DEBUG_print(F("!I:Could not load PID Param"));
		DEBUG_print(F(". Restoring default.\n"));
		setInformation (EE_PID_DEFAULT);
		//Restore HARDCODED parameters but don't save!
		SetTunings(HC_GAIN.Kp.float_00(), HC_GAIN.Ki.float_00(), HC_GAIN.Kd.float_00());
	}
}

void Autopilot::EEPROM_format() {
	DEBUG_print(F("!EEPROM Format..."));

	for (uint16_t i = 0 ; i < EEPROM.length() ; i++) {
	    EEPROM.update(i, 0xFF);
	  }

	DEBUG_print(F("Ok\nStop."));
    while (1) {;}

}

// G, A, M will calibrate one sensor only
// - will calibrate all in secuence G, A, M
// Other value (including 0) will not enter into calibration mode
void Autopilot::EEsave_ReqCal (char sensor)
{

	if (sensor =='0' or sensor =='G' or sensor =='A' or sensor =='M' or sensor =='-') {
		EEPROM.put(EE_address.Flag, sensor); // different to 0, enables Calibration Flag to force calibration
	}

}

char Autopilot::EEload_ReqCal (void)
{
	char sensor;

	EEPROM.get(EE_address.Flag, sensor); // G, A, M or - Require Calibration Flag to force calibration
	if (sensor =='G' or sensor =='A' or sensor =='M' or sensor =='-') {
		sprintf(DEBUG_buffer,"!Cal.Flag: %c\n", sensor);
		DEBUG_print(DEBUG_buffer);
		return char(sensor);
	}
	return '0'; // No calibration
}


bool Autopilot::EEsave_Calib(){
	return BearingMonitor::EEsave_Calib(EE_address.IMU);
}

e_IMU_cal_status Autopilot::EEload_Calib(){
	return BearingMonitor::EEload_Calib(EE_address.IMU);
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
	DEBUG_print(F("!Saving InstParam..."));

    //instParam_CHECK
    EEPROM.put(eeAddress, CHECKvalue);
    eeAddress += sizeof(CHECKvalue);

    // InstParam
	if (HC==true) {
		DEBUG_print(F("!Restored manufacturer values."));
		instParam=HC_INSTPARAM;}
	else {
		Request_instParam(instParam);
	}

    EEPROM.put(eeAddress, instParam);

    DataStored = true;
	DEBUG_print(F("Ok\n"));
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
				instParam.flag.minFeedback && instParam.flag.maxFeedback;
		if (Loaded) Change_instParam (instParam);
		DEBUG_print();

    }
	return Loaded;
}


bool Autopilot::EEsave_PIDgain(bool HC){
	bool DataStored=false;
	long eeAddress = EE_address.PIDgain;
	s_PIDgain PIDgain;
	DEBUG_print(F("!Saving PIDgain..."));

    //PID_CHECK
    EEPROM.put(eeAddress, CHECKvalue);
    eeAddress += sizeof(CHECKvalue);

	if (HC==true) {
		PIDgain = HC_PIDGAIN;
		DEBUG_print(F("Restored manufacturer values."));
	} else {
		Request_PIDgain(PIDgain);
	}
    EEPROM.put(eeAddress, PIDgain);

    DataStored = true;
	DEBUG_print(F("!Ok\n"));
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

bool Autopilot::Load_calibrate_py (s_calibrate_py calibrate_py){
	float B[3];
	float Ainv[3][3];
	char sensor;

	B[0]=calibrate_py.GAM_B.x.float_00();
	B[1]=calibrate_py.GAM_B.y.float_00();
	B[2]=calibrate_py.GAM_B.z.float_00();
	Ainv[0][0]=calibrate_py.GAM_Ainv.m11.float_00000();


	Ainv[0][1]=calibrate_py.GAM_Ainv.m12.float_00000();
	Ainv[0][2]=calibrate_py.GAM_Ainv.m13.float_00000();
	Ainv[1][0]=calibrate_py.GAM_Ainv.m21.float_00000();
	Ainv[1][1]=calibrate_py.GAM_Ainv.m22.float_00000();
	Ainv[1][2]=calibrate_py.GAM_Ainv.m23.float_00000();
	Ainv[2][0]=calibrate_py.GAM_Ainv.m31.float_00000();
	Ainv[2][1]=calibrate_py.GAM_Ainv.m32.float_00000();
	Ainv[2][2]=calibrate_py.GAM_Ainv.m33.float_00000();
	sensor = calibrate_py.sensor;
	return BearingMonitor::set_calibrate_py_offsets(B, Ainv, sensor);

}


void Autopilot::printWarning(bool instant) {
	static e_warning prev_warning = NO_WARNING;
	static unsigned long lastWprint = 0;

	if (_lost_W) {
		DEBUG_print(F("!Warning not displayed\n"));
		buzzer_Warning();
		_lost_W = false;
	}

	if ( (instant == true) or (millis() - lastWprint) > W_DISPLAY_TIME ) {

		if (_warning!=NO_WARNING and prev_warning!=_warning) {
			sprintf(DEBUG_buffer,"!WARNING Code: %i\n", _warning);
			DEBUG_print();
			buzzer_Warning();
			prev_warning = _warning;
			_pending_W = false;
		}
		int fmemory = freeMemory();//*100)/8192; //% of free memory in Arduino MEGA (8KB RAM)
		if (fmemory <500) {

			sprintf(DEBUG_buffer,"!Low memory: %i/8192 bytes\n", fmemory );
			DEBUG_print();
		}

		// reset counter
		lastWprint = millis();
	}
	return;
}

void Autopilot::print_PIDFrontend() {
	static unsigned long lastPIDprint = 0;

	int l=6, d=2;
	char c3[l+3];
	if ( (millis() - lastPIDprint) > 1000 ) {
	DEBUG_print(F("PID "));
	  sprintf(DEBUG_buffer,"%s", deblank(dtostrf(PID_ext::getSetpoint(),l,d,c3)));
	  DEBUG_print();
	  DEBUG_print(F(" "));
	  sprintf(DEBUG_buffer,"%s", deblank(dtostrf(this->getInput(),l,d,c3)));
	  DEBUG_print();
	  DEBUG_print(F(" "));
	  sprintf(DEBUG_buffer,"%s", deblank(dtostrf(this->getOutput(),l,d,c3)));
	  DEBUG_print();
	  DEBUG_print(F(" "));
	  sprintf(DEBUG_buffer,"%s", deblank(dtostrf(PID::GetKp(),l,d,c3)));
	  DEBUG_print();
	  DEBUG_print(F(" "));
	  sprintf(DEBUG_buffer,"%s", deblank(dtostrf(PID::GetKi(),l,d,c3)));
	  DEBUG_print();
	  DEBUG_print(F(" "));
	  sprintf(DEBUG_buffer,"%s", deblank(dtostrf(PID::GetKd(),l,d,c3)));
	  DEBUG_print();
	  DEBUG_print(F(" "));
	  if(PID::GetMode()==AUTOMATIC) DEBUG_print(F("Automatic"));
	  else DEBUG_print(F("Manual"));
	  DEBUG_print(F(" "));
	  DEBUG_print(F("Direct\n"));

		// reset counter
		lastPIDprint = millis();
	}

}

