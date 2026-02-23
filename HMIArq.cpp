/*
 * HMIArq.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "HMIArq.h"

HMIArq::HMIArq(Autopilot* Pilot){
	// TODO Auto-generated constructor stub
	MyPilot = Pilot;

}

HMIArq::~HMIArq() {
	// TODO Auto-generated destructor stub
}

void HMIArq::Request_PIDgain(s_PIDgain & PIDgain) {
	MyPilot->Request_PIDgain(PIDgain);
}


void HMIArq::Request_instParam(s_instParam & instParam) {
	MyPilot->Request_instParam(instParam);
}

void HMIArq::Request_APinfo(s_APinfo & APinfo) {

	APinfo.CTS.Towf_00(MyPilot->getTargetBearing());
	APinfo.HDT.Towf_00(MyPilot->getCurrentHeadingT());
	// Deadband value
	APinfo.deadband.Towf_00(MyPilot->getDeadband());
	APinfo.mode=MyPilot->getCurrentMode();
	APinfo.rudder=MyPilot->getTargetRudder();

 }

void HMIArq::Request_IMUcal(s_IMUcal & IMUcal) {

	if (MyPilot->getIMUcheck()==CHECK_ONGOING) {
		IMUcal.IMUcalstatus = 'C';
	} else {
		switch (MyPilot->getIMUstatus()) {
		case CAL_NOT_STARTED:
		case CAL_START:
			IMUcal.IMUcalstatus = '-'; break;
		case CAL_INPROGRESS:
			IMUcal.IMUcalstatus = 'O'; break;
		case CAL_RESULT_RECALIBRATED:
			IMUcal.IMUcalstatus = 'R'; break;
		case CAL_RESULT_NOT_CALIBRATED:
			IMUcal.IMUcalstatus = 'N'; break;
		}
	}

	MyPilot->getCheckSGAM(IMUcal.SYSstatus, IMUcal.GYROstatus, IMUcal.ACCELstatus, IMUcal.MAGNstatus);
	MyPilot->getCheckXYZ(IMUcal.X,IMUcal.Y,IMUcal.Z);

	return;
 }


void HMIArq::Request_FBKcal(s_FBKcal & FBKcal) {
	MyPilot->getFBKcalStatus(FBKcal.cal_minFeedback, FBKcal.cal_maxFeedback);
	return;
 }

void HMIArq::Change_PID(s_PIDgain_flag change, s_gain gain) {
	// if change.K* is true, take value from argument k*. If false, getK*() retrieves current value
	float Kp = (change.Kp ? gain.Kp.float_00() : MyPilot->PID::GetKp());
	float Ki = (change.Ki ? gain.Ki.float_00() : MyPilot->PID::GetKi());
	float Kd = (change.Kd ? gain.Kd.float_00() : MyPilot->PID::GetKd());
	MyPilot->SetTunings (Kp, Ki, Kd);
    MyPilot->buzzer_Beep();
}

void HMIArq::Change_PID_rel (s_PIDgain_flag change, e_operation op, float value ){

	float Kp=MyPilot->PID::GetKp();
	float Ki=MyPilot->PID::GetKi();
	float Kd=MyPilot->PID::GetKd();

	if (change.Kp) operation (op, Kp, value);
	if (change.Ki) operation (op, Ki, value);
	if (change.Kd) operation (op, Kd, value);

	MyPilot->SetTunings (Kp, Ki, Kd);
    MyPilot->buzzer_Beep();

}

void HMIArq::Change_AvgSpeed(int16_t valueAvgSpeed) {
	MyPilot->setAvgSpeed(valueAvgSpeed);
	MyPilot->buzzer_Beep();
}

void HMIArq::Change_CenterRudder(int16_t valueCenterRudder) {
	MyPilot->setDeltaCenterOfRudder(valueCenterRudder);
	MyPilot->buzzer_Beep();
}

void HMIArq::Change_MaxRudder(int16_t value_MaxRudder) {
	MyPilot->setMaxRudder(value_MaxRudder);
	MyPilot->buzzer_Beep();
}

void HMIArq::Change_FbkError(int16_t value_FbkError) {
	MyPilot->setFbkError(value_FbkError);
	MyPilot->buzzer_Beep();
}

void HMIArq::Change_magneticVariation(float value_MagneticVariation) {
	MyPilot->setMagneticVariation(value_MagneticVariation);
	MyPilot->buzzer_Beep();
}

void HMIArq::operation (e_operation op, float &K, float value) {
	switch (op) {
		case OP_ADD:
			K+=value;
			break;
		case OP_DEC:
			K-=value;
			break;
		case OP_MULT:
			K*=value;
			break;
		case OP_DIV:
			K/=value;
			break;
	}
}

void HMIArq::setDBConf (type_DBConfig status) {
	MyPilot->setDBConf (status);
    MyPilot->buzzer_Beep();

}

void HMIArq::nextDBConf () {
	MyPilot->nextDBConf ();
    MyPilot->buzzer_Beep();

}

bool HMIArq::Change_instParam (s_instParam instParam) {
	return MyPilot->Change_instParam (instParam);
    MyPilot->buzzer_Beep();

}

//void HMIArq::Apply_PIDrecom(){
//	if (MyPilot->CopyToPIDAutoTune()) {
//		MyPilot->buzzer_Beep();
//		MyPilot->setCurrentMode(STAND_BY);
//	} else {
//		DEBUG_print ("!PID recommendation not available\n");
//	}
//}


void HMIArq::ResetPID(){
	MyPilot->ResetTunings();
    MyPilot->buzzer_Beep();

}

void HMIArq::Start_Stop(e_start_stop type){
	MyPilot->Start_Stop(type);
	MyPilot->buzzer_Beep();
}

void HMIArq::Start_Stop_wind(void){
	MyPilot->Start_Stop_wind();
	MyPilot->buzzer_Beep();
}

void HMIArq::Enter_Exit_FBK_Calib() {
	MyPilot->Enter_Exit_FBK_Calib();
    MyPilot->buzzer_Beep();
}

void HMIArq::Start_Cancel_AutotunePID() {
	MyPilot->Start_Cancel_AutotunePID();
    MyPilot->buzzer_Beep();
}



void HMIArq::Inc_Rudder_1(){
    MyPilot->changeRudder(+RATE_1);
    MyPilot->buzzer_Beep();
}
void HMIArq::Inc_Rudder_10(){
    MyPilot->changeRudder(+RATE_10);
    MyPilot->buzzer_Beep();
}
void HMIArq::Dec_Rudder_1(){
    MyPilot->changeRudder(-RATE_1);
    MyPilot->buzzer_Beep();
}
void HMIArq::Dec_Rudder_10(){
    MyPilot->changeRudder(-RATE_10);
    MyPilot->buzzer_Beep();
}
void HMIArq::Stop_Rudder(){
    MyPilot->setTargetRudder(MyPilot->getCurrentRudder());//TODO: Should be restricted in ActuatorManager and public in MacuaAutopilot
}
void HMIArq::Inc_Course_1(){
	  MyPilot->setTargetBearing(MyPilot->getTargetBearing()+1);
	  MyPilot->buzzer_Beep();
}
void HMIArq::Inc_Course_10(){
	  MyPilot->setTargetBearing(MyPilot->getTargetBearing()+10);
	  MyPilot->buzzer_Beep();
}
void HMIArq::Inc_Course_100(){
	  MyPilot->setTargetBearing(MyPilot->getTargetBearing()+100);
	  MyPilot->buzzer_Beep();
}
void HMIArq::Dec_Course_1(){
	  MyPilot->setTargetBearing(MyPilot->getTargetBearing()-1);
	  MyPilot->buzzer_Beep();
}
void HMIArq::Dec_Course_10(){
	  MyPilot->setTargetBearing(MyPilot->getTargetBearing()-10);
	  MyPilot->buzzer_Beep();
}
void HMIArq::Dec_Course_100(){
	  MyPilot->setTargetBearing(MyPilot->getTargetBearing()-100);
	  MyPilot->buzzer_Beep();
}

void HMIArq::Set_NewCourse(float newCourse){
	  MyPilot->setTargetBearing(newCourse);
	  MyPilot->buzzer_Beep();
}

void HMIArq::Set_NextCourse(float nextCourse){
	  MyPilot->setNextCourse (nextCourse);
	  MyPilot->buzzer_Beep();
}

void HMIArq::Set_NextCourse_delta(int delta){
	Set_NextCourse (MyPilot->getNextCourse() + delta);
}

void HMIArq::Set_Tacking(int delta){
	Set_NextCourse (MyPilot->getCurrentHeadingT() + delta);
}

void HMIArq::Set_NewDeltaCourse(float newDCourse){
	  MyPilot->setTargetBearing(MyPilot->getTargetBearing()+newDCourse);
	  MyPilot->buzzer_Beep();
}

void HMIArq::Set_HeadingDev(){
	MyPilot->setHeadingDev(MyPilot->getNextCourse() - MyPilot->getCurrentHeadingT() + MyPilot->getHeadingDev());
	MyPilot->setNextCourse(0);
	MyPilot->buzzer_Beep();
}

// External Compass mode functions
//void HMIArq::received_HDT( s_HDT HDT) {
//	MyPilot->HDTreceived(HDT);
//}
void HMIArq::received_HDGT( s_HDG HDG) {
	MyPilot->HDGTreceived(HDG);
}

// Speed received
void HMIArq::received_SOG( s_SOG SOG) {
	MyPilot->SOGreceived(SOG);
}

// Wing mode functions
void HMIArq::received_AWA( s_AWA AWA) {
	MyPilot->AWAreceived(AWA);
}

void HMIArq::received_TWD( s_TWD TWD) {
	MyPilot->TWDreceived(TWD);
}

void HMIArq::change_windMode(void) {
	 if (MyPilot->getCurrentWindMode()==MODE_AWA)
	 {
		 if (MyPilot->setCurrentWindMode(MODE_TWA)) MyPilot->buzzer_Beep();
	 } else
	 {
		 if (MyPilot->setCurrentWindMode(MODE_AWA)) MyPilot->buzzer_Beep();
	 }

}

void HMIArq::Inc_WindDir_1(){
	  MyPilot->setTargetWindDir_delta(1);
	  MyPilot->buzzer_Beep();
}
void HMIArq::Inc_WindDir_10(){
	MyPilot->setTargetWindDir_delta(10);
	  MyPilot->buzzer_Beep();
}
void HMIArq::Dec_WindDir_1(){
	MyPilot->setTargetWindDir_delta(-1);
	  MyPilot->buzzer_Beep();
}
void HMIArq::Dec_WindDir_10(){
	MyPilot->setTargetWindDir_delta(-10);
	  MyPilot->buzzer_Beep();
}

// Track mode functions
void HMIArq::received_APB( s_APB APB) {
	MyPilot->APBreceived(APB);
}

void HMIArq::Accept_Next(void) {
	switch (MyPilot->getCurrentMode())
	{
	case STAND_BY:
	case AUTO_MODE:
	case TRACK_MODE:
		// Accept WPnext
		if (MyPilot->activateWPnext())
		{
			MyPilot->setCurrentMode(TRACK_MODE);
		} else {
			MyPilot->setTargetBearing(MyPilot->getNextCourse());
		}
		break;
	case WIND_MODE:
		MyPilot->set_nextWindDir();
		break;
	default:
		return;
	}
	MyPilot->buzzer_Beep();
}

//Calibration functions
void HMIArq::Start_Cal(char sensor){
	MyPilot->Start_Cal(sensor);
	  MyPilot->buzzer_Beep();
}

void HMIArq::Cal_NextSensor(void){
	MyPilot->Cal_NextSensor();
	  MyPilot->buzzer_Beep();
}

void HMIArq::Cancel_Cal(){
	MyPilot->Cancel_Cal();
}

void HMIArq::Save_Cal(){
	MyPilot->EEsave_Calib();
	  MyPilot->buzzer_Beep();

}

void HMIArq::Save_instParam(){
	MyPilot->EEsave_instParam();
	  MyPilot->buzzer_Beep();

}

void HMIArq::Save_PIDgain(){
	MyPilot->EEsave_PIDgain();
	  MyPilot->buzzer_Beep();

}

void HMIArq::Save_HCParam(){
	MyPilot->EEsave_HCParam();
	  MyPilot->buzzer_Beep();

}

void HMIArq::Load_calibrate_py(s_calibrate_py calibrate_py){
	MyPilot->Load_calibrate_py(calibrate_py);
	  MyPilot->buzzer_Beep();

}


// Check if request has timeout, end request and return true.
// if not return false.
bool HMIArq::updateRequestTimeout() {
	if ((_requestStatus==WAITING_USER_ANSWER) && ((millis()-_requestTime)>MAX_USER_ANSWER_TIME)) {
			_requestStatus = NO_USER_REQUEST;

			return true;
		}
	return false;

}

// Polls for user request status.
//If not launched, it LAUNCHES NEW REQUEST!
// If launched and answered, returns answer and RESET REQUEST!
// If launched and not answered, returns status

e_requestStatus HMIArq::userRequest (void) {
	switch (_requestStatus) {
	case NO_USER_REQUEST:
		_requestStatus = WAITING_USER_ANSWER;
		break;
	case USER_ACCEPTED:
		_requestStatus = NO_USER_REQUEST;
		return USER_ACCEPTED;
		break;
	case USER_REJECTED:
		_requestStatus = NO_USER_REQUEST;
		return USER_REJECTED;
		break;
	case WAITING_USER_ANSWER:
		return WAITING_USER_ANSWER;
		break;
	}

	return _requestStatus;
}

// Registers user answer to request (if there is an active request).
// Arguments:
// true --> USER_ACCEPTED
// false--> USER_REJECTED
// Return: request status
e_requestStatus HMIArq::userRequestAnswer (bool answer) {

	if (_requestStatus==WAITING_USER_ANSWER) {
		_requestStatus = (answer? USER_ACCEPTED: USER_REJECTED);
	}
	switch (_requestStatus) {
	case USER_ACCEPTED:
		//DEBUG_print( "User accepted\n" );

		break;
	case USER_REJECTED:
		//DEBUG_print( "User rejected\n" );

		break;
	default:
		//DEBUG_print( "Other\n" );
		break;
	}

	return _requestStatus;
}

e_requestStatus HMIArq::getRequestStatus () {
	return _requestStatus;
}
