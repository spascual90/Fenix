/*
 * IF_NMEA.h
 *
 *  Created on: 30 dic. 2018
 *      Author: Sergio
 */

#ifndef IF_NMEA_H_
#define IF_NMEA_H_

#include "GPSport.h" // Serial NMEA IF Configuration in GPSPort.h not in Fenix.ino!

#ifdef SERIAL_IF_AVAILABLE

#include <Arduino.h>

#include "emcNMEA.h"
#include "HMIArq.h"

#ifndef NMEAGPS_INTERRUPT_PROCESSING
	#error You must define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif


//// C runtime variables
//// -------------------
//extern unsigned int __bss_end;
//extern unsigned int __heap_start;
//extern void *__brkval;

class IF_NMEA: public emcNMEA, public HMIArq {

public:
	IF_NMEA(Autopilot* Pilot);

	//HMIArq I/F implementation
	void setup();
	void refresh();
	void startAllTX();
	void stopAllTX();

	//emcNMEA I/F implementation
	float getMagneticVariation() {	return MyPilot->getMagneticVariation();}
	float getHeadingDev() {	return MyPilot->getHeadingDev();}
    float toMagnetic(float value) {return MyPilot->ChangeRef(value, BearingMonitor::TRUEtoMAGNETIC);}
    float toTrue(float value) {return MyPilot->ChangeRef(value, BearingMonitor::MAGNETICtoTRUE);}
	int getRudder(){return MyPilot->getCurrentRudder();}
	float getHeadingC() {return MyPilot->getCurrentHeadingC();}
	float getHeadingT() {return MyPilot->getCurrentHeadingT();}
	void printPEMC_03(Stream * outStream);
	void printPEMC_05(Stream * outStream);
	void printPEMC_07(Stream * outStream);
	void printPEMC_12(Stream * outStream);
	void printPEMC_13(Stream * outStream);
	void printAPB(Stream * outStream, s_APB APB);


private:
	void refresh_INorder();
	void TESTER_sincroTime(void);

};



#endif
#endif

/* IF_NMEA_H_ */
