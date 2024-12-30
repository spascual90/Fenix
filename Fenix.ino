//header TODO

// v0.1 - Initial Beta
// V0.2
// PWM and DIR PIN changed to ensure compatibility with LCDKeyPad


#include <Arduino.h>

// Comment any IF not implemented. Activating an IF without HW device installed will cause unexpected effects
//#define HMI_LCDKEYPAD // LCDKeypad IF
#include "GPSport.h" // Serial NMEA,Debug and Bluetooth IF Configuration in GPSPort.h not in Fenix.ino!

#ifdef HMI_BT

#include "BT.h"
#endif

#ifdef HMI_LCDKEYPAD
#include "LCDKeyPad.h"
#endif

#ifdef SERIAL_IF_AVAILABLE
#include "IF_NMEA.h"

#ifndef NMEAGPS_INTERRUPT_PROCESSING
	  #error You must define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

#endif

#include "Autopilot.h"

// INSTANTIATE OBJECTS
Autopilot MyPilot;

#ifdef HMI_LCDKEYPAD
LCDKeyPad MyLCDKP(&MyPilot);
#endif

#ifdef HMI_BT
BT MyBT(&MyPilot);
#endif

#ifdef SERIAL_IF_AVAILABLE
static IF_NMEA parserNMEA(&MyPilot); // This parses received characters

static void NMEAisr( uint8_t c )
{
  parserNMEA.handle( c );

} // GPSisr
#endif

void setup()
{

	// Setup NMEA IF if defined
#ifdef SERIAL_IF_AVAILABLE
	parserNMEA.setup();

	gpsPort.attachInterrupt( NMEAisr );
	gpsPort.begin( 9600 );
#endif

	// Setup LCDKeypad if defined
	#ifdef HMI_LCDKEYPAD
	MyLCDKP.setup();
	#endif

	// Setup Bluetooth if defined
	#ifdef HMI_BT
	MyBT.setup();
	#endif

	// Setup autopilot
	switch (MyPilot.setup()){
		case SETUP_OK:
			DEBUG_print(F("Start\n"));
			break;

		case IMU_ERROR:
		case FEEDBACK_ERROR:
			MyPilot.buzzer_Error();
			DEBUG_print(F("Stop\n"));
			while(1){;}
			break;
	}


#ifdef SERIAL_IF_AVAILABLE
    //Start transmission of NMEA/PEMC messages
    parserNMEA.startAllTX();
#endif

}

void loop()
{
	// Refresh NMEA IF if defined
	#ifdef SERIAL_IF_AVAILABLE
	parserNMEA.refresh();
	#endif

	// Refresh LCDKeypad if defined
	#ifdef HMI_LCDKEYPAD
	MyLCDKP.refresh();
	#endif

	// Refresh Bluetooth if defined
	#ifdef HMI_BT
	MyBT.refresh();
	#endif

	// New pilot iteration
	if (MyPilot.Compute() == RUNNING_ERROR) {


		// pilot error. Turn light on and beep
		#ifdef HMI_LCDKEYPAD
		MyLCDKP.turnLight(ON);
		//MyLCDKP.print("PILOT: Error");
		#endif

		//TODO: beep
		}

}
