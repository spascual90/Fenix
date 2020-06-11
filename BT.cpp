 /* BT.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "BT.h"

 BT::BT(Autopilot* Pilot)
 :HMIArq(Pilot)
{
	// TODO Auto-generated constructor stub
	MyPilot = Pilot;
}

BT::~BT() {
	// TODO Auto-generated destructor stub
}

void BT::setup() {
#ifdef BTPort
	  //DEBUG=true;               // set this value TRUE to enable the serial monitor status

	  //SPM original: Serial1.begin(9600);               // Enable this line if you want to use hardware serial (Mega, DUE etc.)
	  //SPM modified:
	  BTPort.begin(9600);//(38400);               // Enable this line if you want to use hardware serial (Mega, DUE etc.)

	  // Use virtuino.vPinMode instead default pinMode method for digital input or digital output pins.
	  // Don't use vPinMode for PWM pins, or pins that their values change many times per second
	  // Every time the value of a vPinMode  pin is changed the virtuino library sends a message and inform virtuino about the pin value.
	  // If an PWM pin is declared as vPinMode pin the arduino will continuously try  to send data to android device because the value of a pwm pin changes constantly
	  // vPinMode(13,OUTPUT);

	  while (!BTPort)
	  	  ;

	  	DEBUG_print( "Bluetooth int... Started\n");
	  	DEBUG_print( "Serial BT device on " BT_PORT_NAME "\n");
	  	DEBUG_print( "Virtuino library version 1.63\n");
	  	DEBUG_print( "Supports Virtuino app ver 1.2.0 or higher\n");

	  	//while(true)	{ATmode();}

#endif
}

void BT::refresh() {
	float M[MAX_AI], temp=0;

	//Float Virtual PIN
	M[AI_HEADING] = (MyPilot->isHeadingValid()? MyPilot->getCurrentHeading():888);
	//M[AI_HEADING] = MyPilot->getCurrentHeading();
	M[AI_TARGET] = MyPilot->getTargetBearing();
	M[AI_DELTA] = MyPilot->getInput();
	M[AI_RUDDER] = MyPilot->getCurrentRudder();
	M[AI_KPCONTRIB] = float (MyPilot->getKpContrib());
	M[AI_ITERM] = float (MyPilot->getITerm());
	M[AI_KDCONTRIB] = float(MyPilot->getKdContrib());
	M[AI_PIDOUT] = float(MyPilot->getOutput());
	M[AI_PREVCTS] = MyPilot->getPrevCourse();
	M[AI_DELTA_CRUDDER] = MyPilot->getDeltaCenterOfRudder();
	M[AI_DEADBAND_VALUE] = MyPilot->dbt.getDeadband();
	M[AI_TRIM_VALUE] = MyPilot->dbt.getTrim();

	uint16_t X = 0;
	int8_t Y = 0;
	uint8_t Z = 0;
	MyPilot->getCheckXYZ(X,Y,Z);
	M[AI_IMU_X] = int(X);
	M[AI_IMU_Y] = int(Y);
	M[AI_IMU_Z] = int(Z);
	uint16_t fbk_min, fbk_max;
	MyPilot->getFBKcalStatus(fbk_min, fbk_max);
	M[AI_FBK_MIN] = fbk_min;
	M[AI_FBK_MAX] = fbk_max;
	// UPDATE VALUES TO APP AND GET BUTTON PRESSED
	updateBT (M);

	updateSpecialBT();

	// Launch action accordingly to button pressed and current mode
	e_APmode currentMode = MyPilot->getCurrentMode();
	switch (getButtonPressed()) {

	//MAIN PANEL
	case BT_START_STOP:
		if (!MyPilot->isCalMode()) {
			if (userRequestAnswer (true)==USER_ACCEPTED){
				//There was a user request, and user accepted
			    MyPilot->buzzer_Beep();
			} else {
				Start_Stop(CURRENT_HEADING); // Accepts request only if necessary
			}
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

    case BT_RETURN_COURSE:
    	userRequestAnswer (false);
    	 Set_NewCourse(MyPilot->getPrevCourse());
		 break;

// BEARING PANNEL
	//Only applicable in STAND_BY, AUTO & TRACK MODE

    case BT_START_STOP_TARGET:
   		Start_Stop(CURRENT_TARGET);
    	break;
    case BT_INC_COURSE_10:
    	Inc_Course_10();
    	break;
     case BT_DEC_COURSE_10:
     	Dec_Course_10();
    	break;
     case BT_INC_COURSE_1:
		Inc_Course_1();
    	break;
     case BT_DEC_COURSE_1:
       	Dec_Course_1();
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

void BT::updateSpecialBT() {

	// SPECIAL OBJECTS IN APP
	// update VIRTUAL DIGITAL LED in APP. AS THERE ARE NOT MANY, ARE TREATED AS SPECIAL BUT EQUIVALENT BEHAVIOUR TO FLOAT VIRTUAL PIN COULD BE IMPLEMENTED
	vDigitalMemoryWrite(DV_LED_DBACTIVE, MyPilot->dbt.getDeadband(MyPilot->getInput())== true ? 1: 0);

	bool S, G, A, M; //Status disabled
	int iS =2, iG=2, iA=2, iM=2;

	if (MyPilot->getCheckSGAM(S, G, A, M)) {
		iS = S;
		iG = G;
		iA = A;
		iM = M;
	}

	vMemoryWrite(AV_LED_IMU_CAL_SYS, iS);
	vMemoryWrite(AV_LED_IMU_CAL_GYRO, iG);
	vMemoryWrite(AV_LED_IMU_CAL_ACEL, iA);
	vMemoryWrite(AV_LED_IMU_CAL_MAGN, iM);

	// update VIRTUAL ANALOG LED in APP. AS THERE ARE NOT MANY, ARE TREATED AS SPECIAL BUT EQUIVALENT BEHAVIOUR TO FLOAT VIRTUAL PIN COULD BE IMPLEMENTED
	vMemoryWrite(AV_LED_STATUS, MyPilot->getCurrentMode());

//	if (HMIArq::getRequestStatus()== WAITING_USER_ANSWER) { //TODO: This should be available in multiple HMI, not only in BT
//		MyPilot->setInformation(NEW_WP_RECEIVED);
//	} else if (MyPilot->getInformation() ==NEW_WP_RECEIVED) {
//		MyPilot->setInformation(NO_MESSAGE);
//	}

	// USER MESSAGES BY PRIORITY: low.ERROR, med.WARNING, hich.INFO
	int message = 0;

	//low. ERROR
	int error = MyPilot->getError();
	if (error!=NO_ERROR) message = error + ERROR_SLOT;

	//med. WARNING
	int warning = MyPilot->getWarning();
	if (warning!=NO_WARNING) message = warning + WARNING_SLOT;

	//high. INFO/USER REQUEST
	int information = MyPilot->getInformation();
	if (information!=NO_MESSAGE) message = information + INFORMATION_SLOT;


	//Only one message displayed, highest priority
	vMemoryWrite(AI_USER_MESSAGE, message);

	vMemoryWrite(AI_KP, MyPilot->GetKp());
	vMemoryWrite(AI_KI, MyPilot->GetKi());
	vMemoryWrite(AI_KD, MyPilot->GetKd());

//	//INDICATIVE SWITCH buttons
//	switch (vDigitalMemoryRead(IS_BLOCK_PID)) {
//			case HIGH:
//				// If unblocked, update AUTOPILOT with APP figures
//				Change_PID(true, true, true, vMemoryRead(AI_KP), vMemoryRead(AI_KI), vMemoryRead(AI_KD));
//				s_gain gain;
//				gain.Kp.Towf_00(vMemoryRead(AI_KP));
//				gain.Ki.Towf_00(vMemoryRead(AI_KI));
//				gain.Kd.Towf_00(vMemoryRead(AI_KD));
//				Change_PID({true, true, true}, gain);
//				break;
//			case LOW:
//				// If blocked, update APP with AUTOPILOT figures
//				vMemoryWrite(AI_KP, MyPilot->GetKp());
//				vMemoryWrite(AI_KI, MyPilot->GetKi());
//				vMemoryWrite(AI_KD, MyPilot->GetKd());
//				break;
//	}
//
//	// SELECTOR SWITCH
//	_k_change.Kp=(vMemoryRead(AI_KSELECT)==0?true:false);
//	_k_change.Ki=(vMemoryRead(AI_KSELECT)==1?true:false);
//	_k_change.Kd=(vMemoryRead(AI_KSELECT)==2?true:false);

	// REGULATOR
	float target = vMemoryRead(AI_DELTA_TARGET);
	if (target!=0) {
		Set_NewCourse(target);
		vMemoryWrite(AI_DELTA_TARGET, 0);
	}

}




