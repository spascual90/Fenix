#ifndef BT_H_
#define BT_H_

#include <Arduino.h>
#include "BTArq.h"
#include "HMIArq.h"

#define FENIX_APP_COMPATIBILITY "v.5.X"

//Virtuino App V6 - Virtual memory button offset (c_BTN_V)

// Emulator arduino board reserved PINS
//M2 	Number of failed connections
//M3 	Enable/ disable BT server
//M4	Selected panel index
//M5    Virtuino App MAIN-version
//M6    Virtuino App SUB-version

//DV0


// PUSH buttons BT_*
// INDICATIVE SWITCH IS_*
// Status LED LED_*

//Digital Virtual PIN (0/1 values) reserved for PUSH BUTTONS
// VIRTUINO APP V6: Virtual memory for buttons in Virtuino App V6 has an offset defined by c_BTN_V
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
		, BT_RESET_PID= 	12 // DEPRECATED?
		, BT_START_STOP_TARGET=	13
		, BT_SET_HEADING_DEV =14
		, BT_START_IMU_CAL=15
		, BT_START_STOP_FBK_CAL=16
		, BT_SAVE_IMU_CAL=17
		, BT_SAVE_FBK_CAL=18
		, BT_SET_VWR = 19
		, BT_RESET_HEADING_DEV = 20
		, BT_SET_CENTER_RUDDER = 21
		, BT_RESTORE_PID = 22
		, BT_STOP_AUTOTUNE = 23 // DEPRECATED
		, BT_SET_DBCONF = 24
		, BT_SAVE_PID = 25

		, MAX_BT = BT_SAVE_PID +1
	};


//Analog Virtual PIN (Float values)
// note, all values between START_AI and MAX_AI shall be defined!
enum e_BT_AI_PIN {
  START_AI = 	-1
, AI_NEXT_CTS= 	0 // Next CTS value
, AI_HEADING= 	1 // Heading value
, AI_CTS= 	2 // CTS value
, AI_DELTA=		3
, AI_RUDDER=	4
, AI_KPCONTRIB= 5
, AI_ITERM=		6
, AI_KDCONTRIB= 7
, AI_PIDOUT=	8
, AI_DELTA_CRUDDER=	9
, AI_DEADBAND_VALUE = 10
, AI_MAGNETIC_VARIATION = 11
, AI_KP = 12
, AI_KI = 13
, AI_KD = 14
, AI_FBK_MIN = 15
, AI_FBK_MAX = 16
, VD_USER_CENTER_RUDDER = 17 //, AI_IMU_X = 17
, VD_USER_MAGNETIC_VARIATION = 18 //, AI_IMU_Y = 18
, AI_HEAD_ALIGN = 19 //AI_IMU_Z = 19
, AI_DELTA_TARGET = 20		// Special AI PINS rotary regulator (user defined)
, AI_USER_MESSAGE = 21		// Special AI PINS
, AV_LED_DBACTIVE = 22		// Special AV PINS
, AI_WARNING = 23			// Special AV PINS
, AI_DELTA_CTS = 24			// Special AV PINS img:CTS (red drop)
, AI_DELTA_NEXT_CTS = 25	// Special AV PINS img: NEXT_CTS2 (white drop)
, AV_LED_STATUS = 26		// Special AV PINS
, AV_LED_IMU_CAL_GYRO = 27	// Special AV PINS
, AV_LED_IMU_CAL_ACEL = 28	// Special AV PINS
, AV_LED_IMU_CAL_MAGN = 29	// Special AV PINS
, AV_LED_IMU_CAL_SYS = 30	// Special AV PINS
, AI_INV_HDG = 31 // img:compass5 (compass heading bow, not north!)
, AI_DELTA_VWR = 32 // img: vwr01 (wind dir rel to bow - white cloud)
, AI_SPEED_REF = 33 // Ref.speed for PID tunning
, AI_SOG = 34 // Current speed
, AI_WIND_SPEED = 35 // Wind speed
, VD_USER_AVG_SPEED = 36
, VD_USER_KP = 37
, VD_USER_KI = 38
, VD_USER_KD = 39
// máximo el valor de c_BTN_V -1 o cambiar el valor de los botones en la App



, MAX_AI = VD_USER_KD +1
};

const uint8_t c_max_V = MAX_AI;
const uint8_t c_max_DV = MAX_BT;
const uint8_t c_BTN_V = 40; //Virtuino App V6 - Virtual memory button offset


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
