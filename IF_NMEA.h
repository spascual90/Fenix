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
	float getDm() {	return MyPilot->getDm();}
    float getMagnetic(float value) {return MyPilot->ChangeRef(value, BearingMonitor::TRUEtoMAGNETIC);}
    float getTrue(float value) {return MyPilot->ChangeRef(value, BearingMonitor::MAGNETICtoTRUE);}
	int getRudder(){return MyPilot->getCurrentRudder();}
	float getHeading() {return MyPilot->getCurrentHeading();}
	void printPEMC_03(Stream * outStream);
	void printPEMC_05(Stream * outStream);
	void printPEMC_07(Stream * outStream);
	void printPEMC_12(Stream * outStream);
	void printPEMC_13(Stream * outStream);
	void printAPB(Stream * outStream, s_APB APB);


private:
	void refresh_INorder();

//	/*!
//	@function   freeMemory
//	@abstract   Return available RAM memory
//	@discussion This routine returns the ammount of RAM memory available after
//	initialising the C runtime.
//	@param
//	@return     Free RAM available.
//	*/
//	static int freeMemory ( void )
//	{
//	   int free_memory;
//
//	   if((int)__brkval == 0)
//	   free_memory = ((int)&free_memory) - ((int)&__bss_end);
//	   else
//	   free_memory = ((int)&free_memory) - ((int)__brkval);
//
//	   return free_memory;
//	}

};



#endif
#endif

/* IF_NMEA_H_ */
