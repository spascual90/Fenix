 /* BT.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "BT.h"

 BT::BT(Autopilot* Pilot)
 :BTArq()
 ,HMIArq(Pilot)
{
	// TODO Auto-generated constructor stub
	MyPilot = Pilot;
}

BT::~BT() {
	// TODO Auto-generated destructor stub
}

void BT::setup() {
#ifdef BTPort
	  BTPort.begin(9600);//(38400);               // Enable this line if you want to use hardware serial (Mega, DUE etc.)

	  while (!BTPort)
	  	  ;

	  	DEBUG_print( "Bluetooth int... Started\n");
	  	DEBUG_print( "Serial BT device on " BT_PORT_NAME "\n");
	  	DEBUG_print( "Virtuino library version 1.63\n");
	  	DEBUG_print( "Supports Virtuino app ver 1.2.0 or higher\n");

	  	begin(onReceived,onRequested,512);  //Start Virtuino. Set the buffer to 256. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes

#endif
}

void BT::refresh() {

	updateBT();


	updateSpecialBT();

	// UPDATE VALUES TO APP AND GET BUTTON PRESSED
	virtuinoRun();

	// Launch action accordingly to button pressed and current mode
	e_APmode currentMode = MyPilot->getCurrentMode();
	switch (getButtonPressed()) {

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
    case BT_INC_TARGET_100:
    	Set_NextCourse_delta(100);
    	//Inc_Course_10();
    	break;
     case BT_DEC_TARGET_100:
    	 Set_NextCourse_delta(-100);
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
	V[AI_NEXTCTS] = float(MyPilot->getNextCourse());
	V[AI_HEADING] = (MyPilot->isHeadingValid()? MyPilot->getCurrentHeading():888);
	V[AI_TARGET] = MyPilot->getTargetBearing();
	V[AI_DELTA] = MyPilot->getInput();
	V[AI_RUDDER] = MyPilot->getCurrentRudder();
	V[AI_KPCONTRIB] = float (MyPilot->getKpContrib());
	V[AI_ITERM] = float (MyPilot->getITerm());
	V[AI_KDCONTRIB] = float(MyPilot->getKdContrib());
	V[AI_PIDOUT] = float(MyPilot->getOutput());
	V[AI_DELTA_CRUDDER] = MyPilot->getDeltaCenterOfRudder();
	V[AI_DEADBAND_VALUE] = MyPilot->dbt.getDeadband();
	V[AI_TRIM_VALUE] = MyPilot->dbt.getTrim();

	static uint16_t X = 0;
	int8_t Y = 0;
	uint8_t Z = 0;
	MyPilot->getCheckXYZ(X,Y,Z);
	V[AI_IMU_X] = X;
	V[AI_IMU_Y] = Y;
	V[AI_IMU_Z] = Z;

	static uint16_t fbk_min, fbk_max;
	MyPilot->getFBKcalStatus(fbk_min, fbk_max);
	V[AI_FBK_MIN] = fbk_min;
	V[AI_FBK_MAX] = fbk_max;

	static uint8_t S, G, A, M;
	MyPilot->getCheckSGAM(S, G, A, M);
	V[AV_LED_IMU_CAL_SYS] = S;
	V[AV_LED_IMU_CAL_GYRO] = G;
	V[AV_LED_IMU_CAL_ACEL] = A;
	V[AV_LED_IMU_CAL_MAGN] = M;

	V[AV_LED_STATUS] = MyPilot->getCurrentMode();

	// VIRTUAL DIGITAL PIN in APP
	DV[DV_LED_DBACTIVE] = MyPilot->dbt.getDeadband(MyPilot->getInput())== true ? 1: 0;

	updateSpecialBT();

	BTArq::updateBT();



}

// SPECIAL OBJECTS IN APP
void BT::updateSpecialBT() {



	// USER MESSAGES BY PRIORITY: low.ERROR, med.WARNING, hich.INFO
	int message = 0;

	//INFO
	int information = MyPilot->getInformation();
	if (information!=NO_MESSAGE) message = information + INFORMATION_SLOT;

	//WARNING
	int warning = MyPilot->getWarning();
	if (warning!=NO_WARNING) message = warning + WARNING_SLOT;

	//ERROR
	int error = MyPilot->getError();
	if (error!=NO_ERROR) message = error + ERROR_SLOT;


	//Only one message displayed, highest priority
	V[AI_USER_MESSAGE] = message;

	V[AI_KP] = MyPilot->GetKp();
	V[AI_KI] = MyPilot->GetKi();
	V[AI_KD] = MyPilot->GetKd();

	// REGULATOR
	if (V[AI_DELTA_TARGET]!=0) {
		if (V[AI_DELTA_TARGET]<0) V[AI_DELTA_TARGET]+=360; // transform (-180,180) to (0, 360);
		Set_NextCourse(V[AI_DELTA_TARGET]);
		V[AI_DELTA_TARGET] = 0;
	}

}




