/*
 * LCDKeyPadArq.h
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#ifndef LCDKeyPadArq_H_
#define LCDKeyPadArq_H_

//all buttons
#define KP_NO_BTN 0
#define KP_BTN_UP 1
#define KP_BTN_DOWN 2
#define KP_BTN_LEFT 3
#define KP_BTN_RIGHT 4
#define KP_BTN_SELECT 5

// LCD display definitions
#define  LCD_ROWS  2
#define  LCD_COLS  16

// LCD pin definitions
#define  LCD_RS    8
#define  LCD_ENA   9
#define  LCD_D4    4
#define  LCD_D5    LCD_D4+1
#define  LCD_D6    LCD_D4+2
#define  LCD_D7    LCD_D4+3
#define  LCD_KEYS  KEY_ADC_PORT // use default pin
#define  LCD_LIGHT 10

#define LIGHTTIME 60000
#define ON true
#define OFF false

#include <arduino.h>
#include <LiquidCrystal.h>

class LCDKeyPadArq: public LiquidCrystal {

public:
	LCDKeyPadArq();
	virtual ~LCDKeyPadArq();
	int getButtonPressed();
	void HMILightDelay(long mSec);
	bool HMILightTimer();
	void turnLight (bool);
	bool HMITimer();
	void printLine(const char str[]);


private:
	//Variables for light on/off timer
	unsigned long _DelayLightStart = 0;
	unsigned long _DelayLightTime =0;
	bool _DelayLight = false;

};

#endif /* LCDKeyPadArq_H_ */
