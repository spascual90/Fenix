/*
 * LCDKeyPad.h
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#ifndef LCDKeyPad_H_
#define LCDKeyPad_H_


// all menu items
#define MENU_MIN 1
#define MENU_CHANGE_RUDDER 1
#define MENU_START_STOP 2
#define MENU_CHANGE_BEARING 3
#define MENU_MAX 3

//menu navigation
#define ERROR -1
#define NO_BTN 0
#define BTN_UP 1
#define BTN_DOWN 2
#define BTN_LEFT 3
#define BTN_RIGHT 4
#define BTN_SELECT 5

#define DOUBLECLICK 500

//#include "MacuaAutopilot.h"
#include <arduino.h>
#include "LCDKeyPadArq.h"
#include "HMIArq.h"

class LCDKeyPad: public LCDKeyPadArq, public HMIArq {

public:
	LCDKeyPad(Macua_Autopilot*);
	virtual ~LCDKeyPad();
	void setup();
	void refresh();
	void changeMenu(int* MenuItem, int Delta);
	void displayMenu(int x);
	void printTitle();
	void printInstr(e_actions x);
	int getInstructions();
	void printCalibrationStatus();
	void printRudderStatus();
	void print0(float heading);
	void HMIDelay(long mSec);

private:
	//Variables for light on/off timer
	unsigned long _DelayLightStart = 0;
	unsigned long _DelayLightTime =0;
	bool _DelayLight = false;

	//States for the menu.
	int currentMenuItem = 1;
	int lastState = 1;
	unsigned long _DelayStart = 0;
	unsigned long _DelayTime =0;
	boolean _Delay = false;

};

#endif /* LCDKeyPad_H_ */
