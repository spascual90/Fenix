 /* BT.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "BT.h"

 BT::BT(Autopilot* Pilot)
 :BTArq(c_max_V, c_max_DV, c_BTN_V)
 ,HMIArq(Pilot)
{
	// TODO Auto-generated constructor stub
	MyPilot = Pilot;
}

BT::~BT() {};

void BT::setup() {
#ifdef BTPort
	  BTPort.begin(9600);//(38400);               // Enable this line if you want to use hardware serial (Mega, DUE etc.)

	  while (!BTPort)
	  	  ;

	  	DEBUG_print( "Bluetooth int... Started\n");
	  	DEBUG_print( "Serial BT device on " BT_PORT_NAME "\n");
	  	DEBUG_print( "Supports Fenix App ");
	  	DEBUG_print(FENIX_APP_COMPATIBILITY);
	  	DEBUG_print("\n");

	  	begin(onReceived,onRequested,512);  //Start Virtuino. Set the buffer to 256. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes

#endif
}

void BT::refresh() {

	updateBT();
	virtuinoRun();        // Necessary function to communicate with Virtuino. Client handler
	triggerAction();
}

void BT::triggerAction () {

	uint8_t button = getButtonPressed();

	if (button == BT_NO_BTN) return;

	//sprintf(DEBUG_buffer,"!triggerAction: %i\n", button);
	//DEBUG_print();

	// Launch action accordingly to button pressed and current mode
	e_APmode currentMode = MyPilot->getCurrentMode();

	switch (button) {

	//MAIN PANEL
	case BT_START_STOP:
		if (!MyPilot->isCalMode()) {
			userRequestAnswer (false);
			Start_Stop(CURRENT_HEADING); // Accepts request only if necessary
		}
		break;

   case BT_INC_MIX_10:
		userRequestAnswer (false);
		switch (currentMode) {
		case STAND_BY:
		case CAL_FEEDBACK:
			Inc_Rudder_10();
			break;
		case AUTO_MODE:
		case TRACK_MODE:
			Inc_Course_10();
			break;
		default:
			break;
		}
		break;

    case BT_DEC_MIX_10:
    	userRequestAnswer (false);
    	switch (currentMode) {
    	case STAND_BY:
    	case CAL_FEEDBACK:
    		Dec_Rudder_10();
    		break;
    	case AUTO_MODE:
    	case TRACK_MODE:
    		Dec_Course_10();
    		break;
    	default:
    		break;
    	}
		break;

    case BT_INC_MIX_1:
    	userRequestAnswer (false);
    	switch (currentMode) {
    	case STAND_BY:
    	case CAL_FEEDBACK:
    		Inc_Rudder_1();
    		break;
    	case AUTO_MODE:
    	case TRACK_MODE:
    		Inc_Course_1();
    		break;
    	default:
    		break;
    	}
		break;

    case BT_DEC_MIX_1:
    	userRequestAnswer (false);
    	switch (currentMode) {
    	case STAND_BY:
    	case CAL_FEEDBACK:
    		Dec_Rudder_1();
    		break;
    	case AUTO_MODE:
    	case TRACK_MODE:
    		Dec_Course_1();
    		break;
    	default:
    		break;
    	}
		break;

    case BT_NEXT_COURSE:
    	Accept_Next();
		 break;

// BEARING PANNEL
	//Only applicable in STAND_BY, AUTO & TRACK MODE

    case BT_START_STOP_TARGET:
   		Start_Stop(CURRENT_TARGET);
    	break;
    case BT_TACK_STARBOARD:
    	Set_Tacking(100);
    	break;
     case BT_TACK_PORTBOARD:
    	 Set_Tacking(-100);
    	break;
     case BT_INC_TARGET_10:
    	 Set_NextCourse_delta(10);
    	break;
     case BT_DEC_TARGET_10:
    	 Set_NextCourse_delta(-10);
    	break;
     case BT_USER_REQUEST_ACCEPTED:
    	 userRequestAnswer (true);
    	 break;
     case BT_USER_REQUEST_REJECTED:
    	 userRequestAnswer (false);
		 break;
     case BT_SET_HEADALIGN:
    	 Set_Headalign();
    	 break;
     case BT_SET_VWR:
     	switch (currentMode) {
     	case AUTO_MODE:
     	case WIND_MODE:
 			userRequestAnswer (false);
 			Start_Stop_wind ();
 			break;
     	default:
     		break;
     	}
     	break;

// CONFIGURATION PANEL

    case BT_RESET_PID:
    	ResetPID();
    	break;

 // IMU CALIBRATION PANEL
    case BT_START_IMU_CAL:
		if (currentMode==STAND_BY) Start_Cal();
    	break;

    case BT_SAVE_IMU_CAL:
		if (currentMode==STAND_BY) Save_Cal();
		break;

 // FEEDBACK CALIBRATION PANEL
    case BT_START_STOP_FBK_CAL:
		Enter_Exit_FBK_Calib();//Enter FBK cal mode
    	break;

    case BT_SAVE_FBK_CAL:
		if (currentMode == CAL_FEEDBACK ) Enter_Exit_FBK_Calib();// Exit FBK cal mode
		Save_instParam();

		break;


// Deprecated functions
//    case BT_K_MUL2:
//    	Change_PID_rel (_k_change, OP_MULT, 2 );
//    	break;
//    case BT_K_DIV2:
//    	Change_PID_rel (_k_change, OP_DIV, 2 );
//    	break;
//    case BT_K_MUL10:
//    	Change_PID_rel (_k_change, OP_MULT, 10 );
//    	break;
//    case BT_K_DIV10:
//    	Change_PID_rel (_k_change, OP_DIV, 10 );
//    	break;
    default: //Other values not managed
    	break;
	}
}


void BT::updateBT(){

	//VIRTUAL PIN IN APP (FLOAT)
	_V[AI_NEXT_CTS] = float(MyPilot->getNextCourse());
	_V[AI_HEADING] = (MyPilot->isHeadingValid()? MyPilot->getCurrentHeading():888);
	_V[AI_CTS] = MyPilot->getTargetBearing();
	_V[AI_DELTA] = MyPilot->getInput();
	_V[AI_RUDDER] = MyPilot->getCurrentRudder();
	_V[AI_KPCONTRIB] = float (MyPilot->getKpContrib());
	_V[AI_ITERM] = float (MyPilot->getITerm());
	_V[AI_KDCONTRIB] = float(MyPilot->getKdContrib());
	_V[AI_PIDOUT] = float(MyPilot->getOutput());

	_V[AI_DELTA_CRUDDER] = MyPilot->getDeltaCenterOfRudder();
	_V[AI_DEADBAND_VALUE] = MyPilot->dbt.getDeadband();
	_V[AI_TRIM_VALUE] = MyPilot->dbt.getTrim();

	static uint16_t X = 0;
	int8_t Y = 0;
	uint8_t Z = 0;
	MyPilot->getCheckXYZ(X,Y,Z);
	_V[AI_IMU_X] = X;
	_V[AI_IMU_Y] = Y;
	_V[AI_IMU_Z] = Z;

	static uint16_t fbk_min, fbk_max;
	MyPilot->getFBKcalStatus(fbk_min, fbk_max);
	_V[AI_FBK_MIN] = fbk_min;
	_V[AI_FBK_MAX] = fbk_max;

	static uint8_t S, G, A, M;
	MyPilot->getCheckSGAM(S, G, A, M);
	_V[AV_LED_IMU_CAL_SYS] = S;
	_V[AV_LED_IMU_CAL_GYRO] = G;
	_V[AV_LED_IMU_CAL_ACEL] = A;
	_V[AV_LED_IMU_CAL_MAGN] = M;

	_V[AV_LED_STATUS] = MyPilot->getCurrentMode();

	_V[AI_KP] = MyPilot->GetKp();
	_V[AI_KI] = MyPilot->GetKi();
	_V[AI_KD] = MyPilot->GetKd();

	_V[AV_LED_DBACTIVE] = MyPilot->dbt.getDeadband(MyPilot->getInput())== true ? 1: 0;

	_V[AI_USER_MESSAGE] = MyPilot->getInformation();

//	Information codes (0 TO 7)
//		NO_MESSAGE,
//		NOT_USED,
//		SETUP_INPROGRESS,
//		IMU_RECAL_INPROGRESS,
//		EE_PID_DEFAULT,
//		TRACKMODE_AVAILABLE,
//		TRACKING,
//		CONFIRM_NEW_WP

	_V[AI_WARNING] = MyPilot->getWarning();

// Warning codes (0 TO 7)
//		NO_WARNING,
//		FBK_ERROR_HIGH,
//		OUT_OF_COURSE,
//		EE_INSTPARAM_NOTFOUND,
//		EE_IMU_NOTFOUND,
//		IMU_LOW,
//		WP_INVALID
//		NO_WIND_DATA

	updateSpecialBT();

}

// SPECIAL OBJECTS IN APP
void BT::updateSpecialBT() {
	// WIND ROSE
	_V[AI_INV_HDG] = 360 - _V[AI_HEADING];

	_V[AI_DELTA_CTS] = _V[AI_CTS] - _V[AI_HEADING];
	if (_V[AI_DELTA_CTS]<0) {_V[AI_DELTA_CTS]+= 360;}
	_V[AI_DELTA_CTS] = fmod (_V[AI_DELTA_CTS], double(360));

	_V[AI_DELTA_NEXT_CTS] = _V[AI_NEXT_CTS] - _V[AI_HEADING];
	if (_V[AI_DELTA_NEXT_CTS]<0) {_V[AI_DELTA_NEXT_CTS]+= 360;}
	_V[AI_DELTA_NEXT_CTS] = fmod (_V[AI_DELTA_NEXT_CTS], double(360));

	_V[AI_DELTA_VWR] = float(MyPilot->getWindDir());
	if (_V[AI_DELTA_VWR]==-1) _V[AI_DELTA_VWR]=180; // Hides pointer in App

	// REGULATOR
	if (_V[AI_DELTA_TARGET]!=0) {
		_V[AI_DELTA_TARGET]+=_V[AI_HEADING];
		if (_V[AI_DELTA_TARGET]<0) _V[AI_DELTA_TARGET]+=360; // transform (-180,180) to (0, 360);
		_V[AI_DELTA_TARGET] = fmod (_V[AI_DELTA_TARGET], double(360));

		Set_NextCourse(_V[AI_DELTA_TARGET]);
		_V[AI_DELTA_TARGET] = 0;
	}


}




