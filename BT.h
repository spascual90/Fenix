#ifndef BT_H_
#define BT_H_

#include <Arduino.h>
#include "BTArq.h"
#include "HMIArq.h"

#define FENIX_APP_COMPATIBILITY "v.2.0 and V.2.1"


// Emulator arduino board reserved PINS
//V0	Selected panel index
//V2 	Number of failed connections
//V3 	Enable/ disable BT server
//V4	360-CTS
//V12 Enable / disable App Demo mode
//DV0


// PUSH buttons BT_*
// INDICATIVE SWITCH IS_*
// Status LED LED_*

//Digital Virtual PIN (0/1 values) reserved for PUSH BUTTONS
// note, all enum values between START_BT and MAX_BT shall be defined!
enum e_BT_push_button {
		  START_BT=			-1

		, BT_START_STOP=	0
		, BT_USER_REQUEST_ACCEPTED = 1
		, BT_USER_REQUEST_REJECTED = 2
		, BT_INC_MIX_1=		3
		, BT_INC_MIX_10= 	4
		, BT_DEC_MIX_1= 	5
		, BT_DEC_MIX_10=	6
		, BT_NEXT_COURSE= 7
		, BT_INC_TARGET_10=	8
		, BT_TACK_STARBOARD=	9
		, BT_DEC_TARGET_10=	10
		, BT_TACK_PORTBOARD=	11
		, BT_RESET_PID= 	12
		, BT_START_STOP_TARGET=	13
		, BT_SET_HEADALIGN =14
		, BT_START_IMU_CAL=15
		, BT_START_STOP_FBK_CAL=16
		, BT_SAVE_IMU_CAL=17
		, BT_SAVE_FBK_CAL=18
		, BT_FREE19 = 19
		, BT_FREE20 = 20
		, MAX_BT = BT_FREE20
	};


//Analog Virtual PIN (Float values)
// note, all values between START_AI and MAX_AI shall be defined!
enum e_BT_AI_PIN {
  START_AI = 	-1
, AI_NEXT_CTS= 	0
, AI_HEADING= 	1
, AI_CTS= 	2
, AI_DELTA=		3
, AI_RUDDER=	4
, AI_KPCONTRIB= 5
, AI_ITERM=		6
, AI_KDCONTRIB= 7
, AI_PIDOUT=	8
, AI_DELTA_CRUDDER=	9
, AI_DEADBAND_VALUE = 10
, AI_TRIM_VALUE = 11
, AI_KP = 12
, AI_KI = 13
, AI_KD = 14
, AI_FBK_MIN = 15
, AI_FBK_MAX = 16
, AI_IMU_X = 17
, AI_IMU_Y = 18
, AI_IMU_Z = 19
, AI_DELTA_TARGET = 20		// Special AI PINS
, AI_USER_MESSAGE = 21		// Special AI PINS
, AV_LED_DBACTIVE = 22		// Special AV PINS
, AI_WARNING = 23			// Special AV PINS
, AI_DELTA_CTS = 24			// Special AV PINS
, AI_DELTA_NEXT_CTS = 25		// Special AV PINS
, AV_LED_STATUS = 26		// Special AV PINS
, AV_LED_IMU_CAL_GYRO = 27	// Special AV PINS
, AV_LED_IMU_CAL_ACEL = 28	// Special AV PINS
, AV_LED_IMU_CAL_MAGN = 29	// Special AV PINS
, AV_LED_IMU_CAL_SYS = 30	// Special AV PINS
, AI_INV_HDG = 31
, FREE32 = 32
, MAX_AI = FREE32

};

const uint8_t c_max_V = MAX_AI;
const uint8_t c_max_DV = MAX_BT;

class BT: public BTArq, public HMIArq {

public:
	BT(Autopilot*);
	virtual ~BT();

	//HMIArq I/F implementation
	void setup();
	void refresh();


private:
	Autopilot* MyPilot;
	void updateBT();
	void updateSpecialBT();
	void triggerAction (void);

	s_PIDgain_flag _k_change;

};

#endif /* BT_H_ */
