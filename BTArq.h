#ifndef BTARQ_H_
#define BTARQ_H_

#include <Arduino.h>
#include <VirtuinoCM.h>                           // Include VirtuinoCM library to your code
#include "GPSport.h" // Ports configuration

#define V_memory_count 32          // the size of V memory. You can change it to a number <=255)
extern float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
#define DV_memory_count 32          // the size of V memory. You can change it to a number <=255)
extern float DV[DV_memory_count];           // This array is synchronized with Virtuino DV memory.

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
		, BT_INC_TARGET_100=	9
		, BT_DEC_TARGET_10=	10
		, BT_DEC_TARGET_100=	11
		, BT_RESET_PID= 	12
		, BT_START_STOP_TARGET=	13
		, BT_SET_HEADALIGN =14
		, BT_START_IMU_CAL=15
		, BT_START_STOP_FBK_CAL=16
		, BT_SAVE_IMU_CAL=17
		, BT_SAVE_FBK_CAL=18

		, MAX_BT
		, BT_NO_BTN
		, BT_BTN_REPEATED
	};

extern e_BT_push_button v_button;

// Special DV PINS
#define DV_LED_DBACTIVE			27

//Analog Virtual PIN (Float values)
// note, all values between START_AI and MAX_AI shall be defined!
enum e_BT_AI_PIN {
  START_AI = 	-1
, AI_NEXTCTS= 	0
, AI_HEADING= 	1
, AI_TARGET= 	2
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
, FREE22 = 22
, FREE23 = 23
, FREE24 = 24
, FREE25 = 25
, AV_LED_STATUS = 26		// Special AV PINS
, AV_LED_IMU_CAL_GYRO = 27	// Special AV PINS
, AV_LED_IMU_CAL_ACEL = 28	// Special AV PINS
, AV_LED_IMU_CAL_MAGN = 29	// Special AV PINS
, AV_LED_IMU_CAL_SYS = 30	// Special AV PINS

};

void onReceived(char variableType, uint8_t variableIndex, String valueAsText);
String onRequested(char variableType, uint8_t variableIndex);

class BTArq: public VirtuinoCM {

public:
	BTArq();
	virtual ~BTArq();
	void updateBT(void);
	e_BT_push_button getButtonPressed();
	void setButton(e_BT_push_button);
	e_BT_push_button getRepeatedButton();

	void virtuinoRun();
	void vDelay(int delayInMillis);

private:
	e_BT_push_button _lastButton = BT_NO_BTN;

	boolean debug = false;              // set this variable to false on the final code to decrease the request time.

};


#endif /* BTARQ_H_ */
