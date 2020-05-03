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
		DEBUG_print("WARNING: Check your linear actuator wiring or increase Rudder Damping parameter!\n");
		setWarning(FBK_ERROR_HIGH);
		//return FEEDBACK_ERROR; Not considered an error
		break;
	case OK_VIRTUAL:
		DEBUG_print("Debugging: Virtual actuator enabled\n");
		break;
	case FEEDBACK_OK:
		break;
	}

	// Setup IMU

	if (Bearing_Monitor::setup() == NOT_DETECTED) {
		// There was a problem detecting the IMU ... check your connections */
		DEBUG_print("ERROR: IMU Not detected. Check your wiring or I2C ADDR\n");
		setError(IMU_NOTFOUND);
		return IMU_ERROR;
	}

	//Enter into calibration mode
	setInformation(IMU_RECAL_INPROGRESS);

	if (EEload_ReqCal()) {// TODO: more robust check of boolean value in EEPROM is required.
		EEsave_ReqCal(false); // Set flag to disabled to avoid entering into Calibration mode each time
		DEBUG_print("Calibration mode!\n");
		// Launch calibration
		if (BNO_GetCal(true)==RECALIBRATED) { // Calibrates and checks results
			DEBUG_print("Calibration succeed!\n");
			return SETUP_OK;
		}
	  }

	 //  Get and restore BNO Calibration offsets
	 if (EEload_Calib()==NOT_CALIBRATED) {
		// There was a problem reading IMU calibration values
		EEsave_ReqCal(true); // Set flag to enabled to enter into Calibration mode next time system is reset
		DEBUG_print("Please restart autopilot to enter into Calibration Mode\n"); // System is not reset automatically to avoid recurrent writing EEPROM
		setWarning(EE_IMU_NOTFOUND);
		return IMU_ERROR;
	 }
	// Ensures calibration is valid by recalibrating (without check feature)
	if (BNO_GetCal(false)==NOT_CALIBRATED) {
		DEBUG_print("Restart autopilot and move slightly the IMU.\nIf this Warning appears again, please enter into Calibration Mode to calibrate IMU\n");
		setWarning(IMU_RECAL_FAILED);
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
	ComputeLongLoop();

	// Play buzzer if required
	buzzer_play();

	// Updates current rudder just once each loop. Stores value for later use.
	if (updateCurrentRudder()<0) return RUNNING_ERROR;

	switch (_currentMode) {
	case AUTO_MODE:
	case TRACK_MODE:
		ws = compute_Track_Mode();
		break;
	case CAL_IMU: //<--TODO: DELETE
		break;
	case CAL_FEEDBACK:
		ws = compute_Cal_Feedback();
		break;
	case STAND_BY:
		ws = compute_Stand_By();
		break;
	}

	return ws;
}


e_working_status Autopilot::compute_Cal_Feedback(){
	RudderFeedback::compute_Cal_Feedback();
	return RUNNING_OK;
}

e_working_status Autopilot::compute_Stand_By(){
	if (changeRudder(0)!=1) return RUNNING_ERROR;
	return RUNNING_OK;
}

e_working_status Autopilot::compute_Track_Mode(void){
	float PIDerrorPrima = delta180(getTargetBearing(), Bearing_Monitor::getCurrentHeading());
	if (PIDerrorPrima==-360) return RUNNING_ERROR;
	if (ActuatorManager::Compute(PIDerrorPrima)!=1) return RUNNING_ERROR;
	OCA_Compute (PIDerrorPrima);
	return RUNNING_OK;
}

// RETURN true = Mode changed successfully
// false = Change mode aborted
bool Autopilot::setCurrentMode(e_APmode newMode = STAND_BY) {

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

	//POST-CHANGE MODE
	rt = after_changeMode(_currentMode, preMode);

	// Abort change mode
	if (!rt) {
		_currentMode = prevMode;
		return false;
	}
	return true;
}

void Autopilot::ComputeLongLoop() {
	static int low_quality_data=0;
//	static int a=0;
	// Once each XX loops: Update current course and target bearing (in track mode). Stores value for later use.
	if (IsLongLooptime ()) {
		// 						DARK BLUE 	Target CTS
		//						LIGHT BLUE 	Current HDG
		//						GRAY		Current rudder
//		a++;
//		if (a==10) {
//			int TB = int(getTargetBearing());
//			int CB = int(getCurrentHeading());
//		plot2(DEBUG_PORT, TB, CB);
//		DEBUG_PORT.print("\n");
//		a=0;
//		}


		if (updateHeading()) {
			if (getWarning()==IMU_LOW) setWarning(NO_WARNING);
		} else {
			low_quality_data++;

			if (low_quality_data>MAX_LOW_QDATA) {
				reset_calibration();
				low_quality_data=0;
			}
		}

		if (_currentMode == TRACK_MODE) {
			if (checkAPBTimeout()) {
				setCurrentMode(STAND_BY);
			} else {
				setTargetBearing(_APB.CTS.float_00());
			}
		}
		LongLoopReset();
	}

}

bool Autopilot::reset_calibration(){
	displayCalStatus();
	setCurrentMode(STAND_BY);
	setWarning(IMU_LOW);
	Bearing_Monitor::reset_calibration();
	return true;
}

bool Autopilot::before_changeMode(e_APmode newMode, e_APmode currentMode){
	switch (currentMode) {
	case CAL_FEEDBACK:
		set_calFeedback();
		break;
	}
	return true;
}

bool Autopilot::after_changeMode(e_APmode currentMode, e_APmode preMode) {
	switch (currentMode) {
	case AUTO_MODE:
		// Same behaviour as Track mode
	case TRACK_MODE:
		if (isHeadingValid()) {
			startAutoMode();
		} else {
			return false;
		}
		break;
	case CAL_FEEDBACK:
		start_calFeedback();
		break;
	case CAL_IMU:
		break;
	case STAND_BY:
		stopAutoMode();
		_APB = {}; //TODO: Check if this is a good way to reset structure
		OCA_Compute (0); //Stop off-course alarm if active
		break;
	}


	return true;
}

bool Autopilot::isCalMode(void){
	e_APmode mode = getCurrentMode();
	if (mode==CAL_IMU or mode==CAL_FEEDBACK) {
		return true;
	} else {
		return false;
	}
}

void Autopilot::Start_Stop(e_start_stop type){
	float target =-1;
	e_APmode mode = getCurrentMode();
	if (!isCalMode()) {

		switch (type) {
		case CURRENT_HEADING:
			target = getCurrentHeading();
			setTargetBearing(target);
			break;
		case CURRENT_TARGET:
			target = getTargetBearing();
			break;
		//case RECEIVED_TARGET:
			//target = MyPilot->getReceivedTarget(); //TODO: WP received mode (to be analyzed)
			break;
		}

		setPrevCourse(target);

		switch (mode) {
		// If in STAND_BY --> set AUTO MODE
		case STAND_BY:
			setCurrentMode(AUTO_MODE);
			break;
		// If in TRACK MODE or AUTO MODE --> set STAND_BY
		case TRACK_MODE:
		case AUTO_MODE:
			setCurrentMode(STAND_BY);
			break;
		default:
			break;
		}

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

// OVERLOADED FUNCTIONS
void Autopilot::SetTunings(double Kp, double Ki, double Kd) {
	switch (getCurrentMode()) {
	case CAL_FEEDBACK:
	case CAL_IMU:
		break;
	default:
		ActuatorManager::PID_ext::SetTunings(Kp, Ki, Kd);
		break;
	}
}

int Autopilot::changeRudder(int delta_rudder) {
	int ret =0;
	e_dir dir;
	switch (getCurrentMode()) {
	case CAL_FEEDBACK:
		dir=(delta_rudder>0?EXTEND:RETRACT);
		ret = cal_FBK_move(dir);
		break;
	case CAL_IMU:
		break;
	default:
		setCurrentMode(STAND_BY);
		ret = ActuatorManager::changeRudder(delta_rudder);
	break;
	}

	return ret;
}

void Autopilot::setDBConf (type_DBConfig status) {
	switch (getCurrentMode()) {
	case CAL_FEEDBACK:
	case CAL_IMU:
		break;
	default:
		dbt.setDBConf (status);
	break;
	}
}
bool Autopilot::checkAPBTimeout() {
	if ((getCurrentMode()==TRACK_MODE) && ((millis()-_APBtime)>MAX_APB_TIME)) {

		return true;
		}
	return false;
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
	DEBUG_print("Debugging: SAFETY NOTICE: Buzzer disconnected!\n");
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

void Autopilot::buzzer_tone_start (unsigned long frequency, int duration) {
#ifdef BUZZER
	_buzzFrec = frequency;
	_buzzDur = duration;
	BuzzReset();
#endif
}

void Autopilot::buzzer_noTone() {
#ifdef BUZZER
	noTone(PIN_BUZZER);
	_Buzz=false; // Stop buzzer
#endif
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
bool Autopilot::OCA_Compute (float delta) {
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
		}
		return _offCourseAlarmActive;
	} else l_offCourse = true;// else-->we are out of course

	if (l_offCourse == true and sb_offCourse == false) {// change detected-->start counting
		sb_offCourse = true;
		sd_offCourseStartTime = millis();
	}

	if (_offCourseAlarmActive == false and (millis()-sd_offCourseStartTime)>_offCourseMaxTime) {
		_offCourseAlarmActive = true;
		buzzer_tone_start (1000, 1023);
	}

	return _offCourseAlarmActive;
}


//EEPROM FUNCTIONAL MODULE
void Autopilot::EEPROM_setup() {
	sprintf(DEBUG_buffer,"EEPROM V%i\n", EE_address.ver);
	DEBUG_print();
	if (EEload_Param()) {
		DEBUG_print("Parameters loaded. Ok\n");
	} else {
		DEBUG_print("WARNING: Could not load parameters: Restoring default.\n");
		setWarning (EE_INSTPARAM_NOTFOUND);
		//Restore HARDCODED parameters but don't save!
		SetTunings(HC_GAIN.Kp.float_00(), HC_GAIN.Ki.float_00(), HC_GAIN.Kd.float_00());
		Change_instParam (HC_INSTPARAM);
	}
}

void Autopilot::EEsave_ReqCal (bool reqCalib)
{
	EEPROM.put(EE_address.Flag, reqCalib); // true enables Calibration Flag to force calibration
}

bool Autopilot::EEload_ReqCal (void)
{
	bool reqCalib;
	EEPROM.get(EE_address.Flag, reqCalib); // true Require Calibration Flag to force calibration
	return reqCalib;
}


bool Autopilot::EEsave_Calib(){
	bool DataStored = false;
    int eeAddress = EE_address.IMU;

	//DATA TO SAVE
	long bnoID;
	adafruit_bno055_offsets_t Calib;

	DEBUG_print("Saving Calibration...");
    bnoID = getSensorId();
	Calib = getSensorOffsets();

    EEPROM.put(eeAddress, bnoID);
    eeAddress += sizeof(long);
    EEPROM.put(eeAddress, Calib);
    //TODO: implement CRC
    byte crc ;//= CRC8(instParam, size);
    eeAddress += sizeof(Calib);
    EEPROM.put(eeAddress, crc);
    DataStored = true;

	DEBUG_print("Ok\n");
    return DataStored;
}  //  end EEsave_Calib

e_IMU_status Autopilot::EEload_Calib(){
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
		DEBUG_print("WARNING: No Calibration Data for this sensor found!\n");
		return NOT_CALIBRATED;
	}
	DEBUG_print("Found Calibration data...\n");
	eeAddress += sizeof(long);
	EEPROM.get(eeAddress, calibrationData);
	setIniCalib(calibrationData);
	displaySensorOffsets();

	return RECALIBRATED;
}


bool Autopilot::EEsave_HCParam(){
	bool inst_OK, PID_OK;
	inst_OK = EEsave_instParam(true);
	PID_OK = EEsave_PIDgain(true);
	return (inst_OK && PID_OK);
}

bool Autopilot::EEload_Param(){
	bool inst_OK, PID_OK;
	inst_OK = EEload_instParam();
	PID_OK = EEload_PIDgain();
	return (inst_OK && PID_OK);
}

bool Autopilot::EEsave_instParam(bool HC){
	bool DataStored=false;
	//DATA TO SAVE
	s_instParam instParam;
	int eeAddress = EE_address.InstParam;
	DEBUG_print("Saving InstParam...");
	if (HC==true) {
		DEBUG_print("Restored manufacturer values.");
		instParam=HC_INSTPARAM;}
	else {
		Request_instParam(instParam);
	}
    EEPROM.put(eeAddress, instParam);
    //TODO: IMPLEMENT CRC
    //size_t size = sizeof(instParam);
    //byte * direccion = &instParam;
    byte crc ;//= CRC8(instParam, size);
    eeAddress+=sizeof(instParam);
    EEPROM.put(eeAddress, crc);

    DataStored = true;
	DEBUG_print("Ok\n");
    return DataStored;
}

bool Autopilot::EEload_instParam (void){
	bool Loaded = false;
	s_instParam instParam;
	EEPROM.get(EE_address.InstParam, instParam);
	Loaded = instParam.isValid &&
			instParam.flag.avgSpeed && instParam.flag.centerTiller &&
			instParam.flag.headAlign && instParam.flag.instSide && instParam.flag.magVariation &&
			instParam.flag.maxRudder && instParam.flag.offcourseAlarm && instParam.flag.rudDamping &&
			instParam.minFeedback && instParam.maxFeedback;
	if (Loaded) Change_instParam (instParam);

	return Loaded;
}


bool Autopilot::EEsave_PIDgain(bool HC){
	bool DataStored=false;
	int eeAddress = EE_address.PIDgain;
	s_PIDgain PIDgain;
	DEBUG_print("Saving PIDgain...");
	if (HC==true) {
		PIDgain = HC_PIDGAIN;
		DEBUG_print("Restored manufacturer values.");
	} else {
		Request_PIDgain(PIDgain);
	}
    EEPROM.put(eeAddress, PIDgain);
    //TODO: implement CRC
    eeAddress+=sizeof(PIDgain);
    byte crc ;//= CRC8(instParam, size);
    EEPROM.put(eeAddress, crc);
    DataStored = true;
	DEBUG_print("Ok\n");
    return DataStored;
}

bool Autopilot::EEload_PIDgain (void){
	bool Loaded = false;
	s_PIDgain PIDgain;
	EEPROM.get(EE_address.PIDgain, PIDgain);
	Loaded = PIDgain.isValid &&
			PIDgain.flag.DBConfig &&
			PIDgain.flag.sTime &&
			PIDgain.flag.gain.Kp && PIDgain.flag.gain.Ki && PIDgain.flag.gain.Kd;

	if (Loaded) {
		SetTunings(PIDgain.gain.Kp.float_00(), PIDgain.gain.Ki.float_00(), PIDgain.gain.Kd.float_00());
		setDBConf(PIDgain.DBConfig);
	}

	return Loaded;
}

//CRC-8 - based on the CRC8 formulas by Dallas/Maxim
//code released under the therms of the GNU GPL 3.0 license
byte Autopilot::CRC8(const byte *data, size_t dataLength)
{
  byte crc = 0x00;
  while (dataLength--)
  {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--)
   {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum)
     {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

