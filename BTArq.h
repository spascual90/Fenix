#ifndef BTARQ_H_
#define BTARQ_H_

#include <Arduino.h>
#include <VirtuinoBluetooth.h>                           // Include VirtuinoBluetooth library to your code
#include "GPSport.h" // Ports configuration

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
		, BT_RETURN_COURSE= 7
		, BT_INC_COURSE_1=	8
		, BT_INC_COURSE_10=	9
		, BT_DEC_COURSE_1=	10
		, BT_DEC_COURSE_10=	11
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

// Special DV PINS
#define DV_LED_DBACTIVE			27


// Special Analog Virtual PINS
#define AV_LED_STATUS 			26
#define AV_LED_IMU_CAL_GYRO		27
#define AV_LED_IMU_CAL_ACEL		28
#define AV_LED_IMU_CAL_MAGN		29
#define AV_LED_IMU_CAL_SYS		30

//Analog Virtual PIN (Float values)
// note, all values between START_AI and MAX_AI shall be defined!
enum e_BT_AI_PIN {
  START_AI = 	-1
, AI_PREVCTS= 	0
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
, MAX_AI
};

// Special A PINS
#define AI_DELTA_TARGET 20
#define AI_USER_MESSAGE 21

class BTArq: public VirtuinoBluetooth {

public:
	BTArq();
	virtual ~BTArq();
	void updateBT(float FM[]);
	e_BT_push_button getButtonPressed();
	void setButton(e_BT_push_button);
	e_BT_push_button getRepeatedButton();
	int getStatus (e_BT_push_button) const;

private:
	e_BT_push_button _button, _lastButton;
};

#endif /* BTARQ_H_ */
