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
	if (IsTXtime()) {
		switch (MyPilot->getCurrentMode()) {
		case CAL_IMU_MINIMUM:
		case CAL_IMU_COMPLETE:
			printPEMC_12(& gpsPort);
			break;
		case CAL_FEEDBACK:
			printPEMC_13(& gpsPort);
			break;
		default:
			// Only transmits HDM/HDG when no valid HDM message has been received.
			if (MyPilot->isHeadingValid() and !MyPilot->isExtHeading()) {
				printHDG(& gpsPort);
				printHDM(& gpsPort);
			}
			printRSA(& gpsPort);
		}

		fl= true;
		TXReset();
	}

	if (IsTX1time() and !MyPilot->isCalMode()) {
		printPEMC_03(& gpsPort);
		printPEMC_05(& gpsPort);
		printPEMC_07(& gpsPort);
		printPEMC_12(& gpsPort);

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

	// Launch action accordingly to action received and current mode
	e_APmode currentMode = MyPilot->getCurrentMode();

	if (this->INorder.isValid){
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
		case DEC_COURSE_1:
				Dec_Course_1();
			break;
		case DEC_COURSE_10:
				Inc_Course_10();
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
			// FLAGS SOLO SE ACTUALIZAN CON K*, para DBConfig no es necesario porque es el �ltimo campo del mensaje y funciona
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

		case EXT_HEADING: //HDM received
			if (!MyPilot->isCalMode()) {
				received_HDM(INorder.HDM);
			}
			break;

		case RELATIVE_WIND: //VWR received
			if (!MyPilot->isCalMode()) {
				received_VWR(INorder.VWR);
			}
			//DEBUG_print("!VWR\n");

			break;

		case START_CAL:
			switch (currentMode) {
			case STAND_BY:
				delay(500);//TODO: delay necessary?
				Start_Cal();
				break;
			//case CAL_IMU_COMPLETE:
			//case CAL_IMU_MINIMUM:
				//TODO: Implement cancel method
				//delay(500);
				//Cancel_Cal();
			//	break;
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
			if (currentMode==STAND_BY) Save_PIDgain();

			break;
		case SAVE_INST:
			if (currentMode==STAND_BY) Save_instParam();
			break;

		case SAVE_CAL:
			if (currentMode==STAND_BY) Save_Cal();
			break;

		case SAVE_HC: 	// Save HARDCODED PIDgain and instParam to EEPROM
			if (currentMode==STAND_BY) Save_HCParam();
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


#endif //SERIAL_IF_AVAILABLE
