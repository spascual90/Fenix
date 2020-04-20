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
	sprintf(DEBUG_buffer, "Dm =%s\n", dtostrf(getDm(),l,d,c4));
	DEBUG_print();
#else
	DEBUG_print ( "NMEA int... Not defined\n" );
#endif
	DEBUG_PORT.flush();
}


void IF_NMEA::refresh(){
	if (this->overrun()) {
	    this->overrun( false );
	    DEBUG_print( "DATA OVERRUN: took too long to print NMEA data!" );
	  }

	refresh_INorder ();

#ifdef TXNMEA
	bool fl = false;

	if (IsTXtime() ) {
		switch (MyPilot->getCurrentMode()) {
		case STAND_BY:
		case AUTO_MODE:
		case TRACK_MODE:
			if (MyPilot->isHeadingValid()) {
				printHDG(& gpsPort);
				printHDM(& gpsPort);

			}
			printRSA(& gpsPort);

			break;
		case CAL_IMU:
			break;
		case CAL_FEEDBACK:
			break;
		}
		fl= true; //
		TXReset();
	}


	if (IsTX1time() ) {
		switch (MyPilot->getCurrentMode()) {
		case STAND_BY:
		case AUTO_MODE:
		case TRACK_MODE:
			printPEMC_03(& gpsPort);
			printPEMC_05(& gpsPort);
			printPEMC_07(& gpsPort);
			break;
		case CAL_IMU:
			break;
		case CAL_FEEDBACK:
			break;
		}
		fl= true;
		TX1Reset();
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

	HMIArq::updateRequestTimeout(); // Check if request reached timeout and cancel
	// Launch action accordingly to action received and current mode
	e_APmode currentMode = MyPilot->getCurrentMode();
	if (this->INorder.isValid){
		switch (this->INorder.get_order()){
		case START_STOP:
			switch (currentMode) {
			case STAND_BY:
			case AUTO_MODE:
			case TRACK_MODE:
			Start_Stop();
			break;
			}
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
		case DEC_COURSE_1:
				Dec_Course_1();
			break;
		case DEC_COURSE_10:
				Inc_Course_10();
			break;
		case REQ_INST: //$PEMC,08,I*5A received
			switch (currentMode) {
			case CAL_FEEDBACK:
			case CAL_IMU:
				break;
			default:
				printPEMC_03(& gpsPort);
				break;
			}
			break;
		case SET_INST:
			if (Change_instParam (INorder.instParam)) printPEMC_03(& gpsPort); //TODO: Cambiar flags en INorder
			break;

		case REQ_GAIN: //$PEMC,08,G*54 received
			switch (currentMode) {
			case CAL_FEEDBACK:
			case CAL_IMU:
				break;
			default:
				printPEMC_05(& gpsPort);
				break;
			}
			break;

		case SET_GAIN: //$PEMC,06 received
			// FLAGS SOLO SE ACTUALIZAN CON K*, para DBConfig no es necesario porque es el último campo del mensaje y funciona
			Change_PID(INorder.PIDgain.flag.gain, INorder.PIDgain.gain);
			setDBConf (INorder.PIDgain.DBConfig);
			// Change PID time--> Not allowed as may impact in pilot performance
			printPEMC_05(& gpsPort);
			break;

		case REQ_INFO: //$PEMC,08,A*52 received
			switch (currentMode) {
			case CAL_FEEDBACK:
			case CAL_IMU:
				break;
			default:
				printPEMC_07(& gpsPort);
				break;
			}
			break;


		case REQ_TRACK: //APB received
			switch (currentMode) {
			case CAL_FEEDBACK:
			case CAL_IMU:
				break;
			default:
				// Treats received APB info. Only changes Waypoint or mode after valid user confirmation
				if (received_APB(INorder.APB)) {
					MyPilot->setCurrentMode(TRACK_MODE);
					printAPB(& DEBUG_PORT);
				}
				break;
			}
			break;
		case START_CAL:
			switch (currentMode) {
			case STAND_BY:

				Start_Cal();
				break;
			default:
				break;
			}
			break;

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
			switch (currentMode) {
			case STAND_BY:
				Save_PIDgain();
				break;
			default:
				break;
			}
			break;
		case SAVE_INST:
			switch (currentMode) {
			case STAND_BY:
				Save_instParam();
				break;
			default:
				break;
			}
			break;

		case SAVE_CAL:
			switch (currentMode) {
			case STAND_BY:
				Save_Cal();
				break;
			default:
				break;
			}
			break;

		case SAVE_HC: 	// Save HARDCODED PIDgain and instParam to EEPROM
			switch (currentMode) {
			case STAND_BY:
				Save_HCParam();
				break;
			default:
				break;
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

void IF_NMEA::printAPB(Stream * outStream) {
	//load APB into OUTorder
	OUTorder.APB = MyPilot->getAPB();
	OUTorder.APB.isValid= true;
	emcNMEA::printAPB(outStream);
}


#endif //SERIAL_IF_AVAILABLE





