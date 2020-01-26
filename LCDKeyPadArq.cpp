/*
 * LCDKeyPadArq.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "LCDKeyPadArq.h"

LCDKeyPadArq::LCDKeyPadArq()
: LiquidCrystal (LCD_RS, LCD_ENA, LCD_D4, LCD_D5, LCD_D6, LCD_D7)
{
	// TODO Auto-generated constructor stub
}

LCDKeyPadArq::~LCDKeyPadArq() {
	// TODO Auto-generated destructor stub
}

int LCDKeyPadArq::getButtonPressed () {

	int state = KP_NO_BTN;
	int x = analogRead (0);

	//Check analog values from LCD Keypad Shield
	if (x < 100) {
	    //Right
		state = KP_BTN_RIGHT;
	  } else if (x < 200) {
	   //Up
	    state = KP_BTN_UP;
	  } else if (x < 400){
	   //Down
	    state = KP_BTN_DOWN;
	  } else if (x < 600){
	    //left
		state = KP_BTN_LEFT;
	  } else if (x < 800){
	    //Select
	    state = KP_BTN_SELECT;
	  }
	return state;
}

void LCDKeyPadArq::printLine(const char str[]) {
	setCursor(0,0);
	print (str);
}

void LCDKeyPadArq::HMILightDelay(long mSec) {
	_DelayLight=true;
	_DelayLightStart = millis();
	_DelayLightTime = mSec;
}

bool LCDKeyPadArq::HMILightTimer () {
	// returns true if timer is ON and still RUNNING
	// returns false if timer is OFF or is ON but arrived to the limit TIME

	if ((_DelayLight==true) and ((millis() -_DelayLightStart) < _DelayLightTime)) {
		return true;}

	return false;
}

void LCDKeyPadArq::turnLight(bool Status) {
	digitalWrite(LCD_LIGHT, Status);
	if (Status == ON) {
		HMILightDelay(LIGHTTIME);
	} else {
		_DelayLight=false;
		}
}

