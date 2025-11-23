/*
 * IF_NMEA.cpp
 *
 *  Created on: 30 dic. 2018
 *      Author: Sergio
 */

#include "IF_NMEA.h"

#ifdef SERIAL_IF_AVAILABLE

 IF_NMEA::IF_NMEA(Autopilot* Pilot)
 :HMIArq(Pilot)
{

}

void IF_NMEA::setup(){
	int l=8, d=2;
	char c4[l+3];

#ifdef DEBUG_PORT
	DEBUG_PORT.begin(9600);
	while (!DEBUG_PORT)
	  ;

	DEBUG_print ( "\nDEBUG int... Started\n");
	DEBUG_print ("Serial DEBUG PORT on " DEBUG_PORT_NAME "\n");
#endif

#ifdef gpsPort
	if (!gpsPort) gpsPort.begin(9600); // In case PORT is same to DEBUG and already started
	while (!gpsPort)
	  ;
	DEBUG_print ( "NMEA int... Started\n" );
	DEBUG_print ("Serial NMEA device on " GPS_PORT_NAME "\n" );
	#ifndef TXNMEA
		DEBUG_print ("Debug: No NMEA TX\n");
	#endif

#else
	DEBUG_print ( "NMEA int... Not defined\n" );
#endif
	DEBUG_flush();
}


void IF_NMEA::refresh(){
	if (this->overrun()) {
	    this->overrun( false );
	    DEBUG_print( "DATA OVERRUN: took too long to print NMEA data!" );
	  }

	refresh_INorder ();

#ifdef TXNMEA
	bool fl = false;
	if (IsTXtime()) {
		switch (MyPilot->getCurrentMode()) {
		case CAL_IMU_COMPLETE:
			//If calibration is performed by external application, PMEC12 messages are managed directly by IMU Device
			// eg. ICM20948 uses Fenix py for calibration
			if (!MyPilot->isExternalCalibration()) printPEMC_12(& gpsPort);
			break;
		case CAL_FEEDBACK:
			printPEMC_13(& gpsPort);
			break;
		default:
			switch (getTxorder()) {
			case 0:
				#ifdef PRINT_FREE_MEM
					printMemory(& gpsPort);
				#endif
				printRSA(& gpsPort);
				//#ifdef TESTER_IF
				//	TESTER_sincroTime();
				//#endif

				TXNext();
				break;
			case 1:
				// Only transmits HDT/HDG when no valid HDT message has been received.
				if (MyPilot->isHeadingValid() and !MyPilot->isExtHeading()) printHDG(& gpsPort);
				TXNext();
				break;
			case 2:
				// HDT Deprecated
				//// Only transmits HDM/HDG when no valid HDM message has been received.
				//if (MyPilot->isHeadingValid() and !MyPilot->isExtHeading()) printHDM(& gpsPort);
				#ifdef TESTER_IF
				TESTER_sincroTime();
				#endif
				TXNext();
				break;
			default:
				TXReset();
				break;
			}
			break;
		}
		fl= true;

	}

	if (IsTX1time() and !MyPilot->isCalMode()) {
		//One message per loop
		switch (getTx1order()) {
		case 0:
			printPEMC_03(& gpsPort);
			TX1Next();
			break;
		case 1:
			printPEMC_05(& gpsPort);
			TX1Next();
			break;
		case 2:
			printPEMC_07(& gpsPort);
			TX1Next();
			break;
		case 3:
			printPEMC_12(& gpsPort);
			TX1Next();
			break;
		default:
			TX1Reset();
			break;
		}
		fl= true;

	}

	if (fl) gpsPort.flush();

#endif
}

void IF_NMEA::startAllTX(){
	gpsPort.flush();
	TXReset();
	TX1Reset();
}

void IF_NMEA::stopAllTX(){
	gpsPort.flush();
}


void IF_NMEA::refresh_INorder() {

	// Launch action accordingly to action received and current mode
	e_APmode currentMode = MyPilot->getCurrentMode();
	if (this->INorder.isValid){
//		sprintf(DEBUG_buffer, "Order:%i\n", this->INorder.get_order());
//		DEBUG_print();
		switch (this->INorder.get_order()){
		case START_STOP:
			if (!MyPilot->isCalMode()) Start_Stop();
		break;

		case INC_RUDDER_1:
			Inc_Rudder_1();
			break;
		case INC_RUDDER_10:
			Inc_Rudder_10();
			break;
		case DEC_RUDDER_1:
			Dec_Rudder_1();
			break;
		case DEC_RUDDER_10:
			Dec_Rudder_10();
			break;
		case INC_COURSE_1:
				Inc_Course_1();
			break;
		case INC_COURSE_10:
				Inc_Course_10();
			break;
		case INC_COURSE_100:
				Inc_Course_100();
			break;
		case DEC_COURSE_1:
				Dec_Course_1();
			break;
		case DEC_COURSE_10:
				Dec_Course_10();
			break;
		case DEC_COURSE_100:
				Dec_Course_100();
			break;
		case REQ_INST: //$PEMC,08,I*5A received
			if (!MyPilot->isCalMode()) printPEMC_03(& gpsPort);
			break;
		case SET_INST:
			if (Change_instParam (INorder.instParam)) printPEMC_03(& gpsPort); //TODO: Cambiar flags en INorder
			break;

		case REQ_GAIN: //$PEMC,08,G*54 received
			if (!MyPilot->isCalMode()) printPEMC_05(& gpsPort);
			break;

		case SET_GAIN: //$PEMC,06 received
			// FLAGS SOLO SE ACTUALIZAN CON K*, para DBConfig no es necesario porque es el ultimo campo del mensaje y funciona
			Change_PID(INorder.PIDgain.flag.gain, INorder.PIDgain.gain);
			setDBConf (INorder.PIDgain.DBConfig);
			// Change PID time--> Not allowed as may impact in pilot performance
			printPEMC_05(& gpsPort);
			break;

		case REQ_INFO: //$PEMC,08,A*52 received
			if (!MyPilot->isCalMode()) printPEMC_07(& gpsPort);
			break;

		case REQ_TRACK: //APB received
			if (!MyPilot->isCalMode()) {
				received_APB(INorder.APB);
			}
			break;

			break;
		case EXT_HEADING: //HDG/HDT received
			if (!MyPilot->isCalMode()) {
				received_HDGT(INorder.HDG);
			}
			break;
		case RELATIVE_WIND: //VWR received
			if (!MyPilot->isCalMode()) {
				received_VWR(INorder.VWR);
			}

			break;

//		case START_CAL:
//			switch (currentMode) {
//			case STAND_BY:
//				delay(500);//TODO: delay necessary?
//				Start_Cal();
//				break;
//			//case CAL_IMU_COMPLETE:
//			//case CAL_IMU_MINIMUM:
//				//TODO: Implement cancel method
//				//delay(500);
//				//Cancel_Cal();
//			//	break;
//			default:
//				break;
//			}
//			break;

		case EE_FBK_CAL:
			switch (currentMode) {
			case STAND_BY:
			case CAL_FEEDBACK:
				Enter_Exit_FBK_Calib();
				break;
			default:
				break;
			}
			break;

		case SAVE_GAIN:
			if (currentMode==STAND_BY) Save_PIDgain();

			break;
		case SAVE_INST:
			if (currentMode==STAND_BY) {
				Save_instParam();
				//printPEMC_03(& gpsPort);
			}
			break;

		case SAVE_CAL:
			if (currentMode==STAND_BY) Save_Cal();
			break;

		case SAVE_HC: 	// Save HARDCODED PIDgain and instParam to EEPROM
			if (currentMode==STAND_BY) Save_HCParam();
			break;

		case CAL_GYRO:
			if (currentMode==STAND_BY) Start_Cal('G');
			if (currentMode==CAL_IMU_COMPLETE) Cal_NextSensor();
			break;
		case CAL_ACCEL:
			if (currentMode==STAND_BY) Start_Cal('A');
			if (currentMode==CAL_IMU_COMPLETE) Cal_NextSensor();
			break;
		case CAL_MAGNET:
			if (currentMode==STAND_BY) Start_Cal('M');
			if (currentMode==CAL_IMU_COMPLETE) Cal_NextSensor();
			break;
		case CAL_ALL:
			if (currentMode==STAND_BY) Start_Cal();
			if (currentMode==CAL_IMU_COMPLETE) Cal_NextSensor();
			break;
		case LOAD_calibrate_py:
			//Instruction only valid for IMU ICM20948
			Load_calibrate_py(INorder.calibrate_py);
			break;
		case SOG: //SOG received
			if (!MyPilot->isCalMode()) {
				received_SOG(INorder.SOG);
			}
			break;
		default:
	    	 break;
		}
		this->INorder.reset();
	}
}





void IF_NMEA::printPEMC_03(Stream * outStream) {
	Request_instParam (OUTorder.instParam);
	OUTorder.instParam.isValid= true;
	emcNMEA::printPEMC_03(outStream);
}

void IF_NMEA::printPEMC_05(Stream * outStream) {
	//load gain into OUTorder
	Request_PIDgain(OUTorder.PIDgain);
	OUTorder.PIDgain.isValid= true;
	emcNMEA::printPEMC_05(outStream);
}

void IF_NMEA::printPEMC_07(Stream * outStream) {
	//load gain into OUTorder
	Request_APinfo(OUTorder.APinfo);
	OUTorder.APinfo.isValid= true;
	emcNMEA::printPEMC_07(outStream);
}

void IF_NMEA::printPEMC_12(Stream * outStream) {
	//load IMUcal into OUTorder
	Request_IMUcal(OUTorder.IMUcal);
	OUTorder.IMUcal.isValid= true;
	emcNMEA::printPEMC_12(outStream);
}

void IF_NMEA::printPEMC_13(Stream * outStream) {
	//load FBKcal into OUTorder
	Request_FBKcal(OUTorder.FBKcal);
	OUTorder.FBKcal.isValid= true;
	emcNMEA::printPEMC_13(outStream);
}

void IF_NMEA::printAPB(Stream * outStream, s_APB APB) {
	//load APB into OUTorder
	OUTorder.APB = APB;
	OUTorder.APB.isValid= true;
	emcNMEA::printAPB(outStream);
}

void IF_NMEA::TESTER_sincroTime(void) {
		static unsigned long DelayCalcStart = millis();
		int l=7, d=2;
		char c3[l+3];
		char c4[l+3];
		char c5[l+3];

		if ((millis() -DelayCalcStart) < 100) return;

		sprintf(DEBUG_buffer,"#%ld,%s,%s,%s\n", millis(),dtostrf(MyPilot->getKpContrib(),0,d,c3),dtostrf(MyPilot->getITerm(),0,d,c4),dtostrf(MyPilot->getKdContrib(),0,d,c5));
		DEBUG_print();
		DEBUG_flush();
		DelayCalcStart = millis();
	}


#endif //SERIAL_IF_AVAILABLE
