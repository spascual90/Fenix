#include "BTArq.h"

// SPM original:
// BTArq::BTArq(): VirtuinoBluetooth(Serial1)
// SPM modified
BTArq::BTArq()
: VirtuinoBluetooth(BTPort)
{
	// Code to use HardwareSerial
	// VirtuinoBluetooth virtuino(Serial1);            // enable this line and disable all SoftwareSerial lines
	                                                   // Open VirtuinoBluetooth.h file on the virtuino library folder ->  disable the line: #define BLUETOOTH_USE_SOFTWARE_SERIAL

}

BTArq::~BTArq() {
	// TODO Auto-generated destructor stub
}

void BTArq::updateBT (float FM[]) {
	//	   *  void vDigitalMemoryWrite(int digitalMemoryIndex, int value)   write a value to a Virtuino digital memory   (digitalMemoryIndex=0..31, value range = 0 or 1)
	//	   *  int  vDigitalMemoryRead(int digitalMemoryIndex)               read  the value of a Virtuino digital memory (digitalMemoryIndex=0..31, returned value range = 0 or 1)
	//	   *  void vMemoryWrite(int analogMemoryIndex, float value)         write a value to Virtuino float memory       (memoryIndex=0..31, value range as float value)
	//	   *  float vMemoryRead(int analogMemoryIndex)                      read the value of  Virtuino analog memory    (analogMemoryIndex=0..31, returned value range = 0..1023)
	//	   *  run()                                                         neccesary command to communicate with Virtuino android app  (on start of void loop)
	//	   *  void vDelay(long milliseconds);                               Pauses the program (without block communication) for the amount of time (in miliseconds) specified as parameter
	//	   *  int getPinValue(int pin)                                      read the value of a Pin. Usefull for PWM pins
    //------ avoid to use delay() function in your code. Use the command virtuino.vDelay() instead of delay()
	bool reset_button= false;
	e_BT_push_button button = BT_NO_BTN;

	run();            //  necessary command to communicate with Virtuino android app


	// EVALUATE PUSH BUTTONS STATUS IN APP
	int i= int (START_BT);
	while (++i != int(MAX_BT)) {
		if (vDigitalMemoryRead(i)==HIGH) {
			button = e_BT_push_button(i);
			reset_button=true;
		}
	}
	setButton(button);

	// UPDATE TO APP:
	//Reset buttons if necessary
	if (reset_button==true){
		i= int (START_BT);
		while (++i != int(MAX_BT)) {
			vDigitalMemoryWrite(i, LOW);
		}
	}

	//Write FLOAT VPIN
	i= int (START_AI);
	while (++i != int(MAX_AI)) {
		vMemoryWrite(i, FM[i]);
	}


}


// Return last pressed button and empty buffer
// Repeated button are informed
e_BT_push_button BTArq::getButtonPressed()  {
	if (_lastButton == _button) return BT_BTN_REPEATED;
	_lastButton = _button;
	_button = BT_NO_BTN;
	return _lastButton;
}

void BTArq::setButton(e_BT_push_button button) {
	_button = button;
}

// return button when button is repeated
e_BT_push_button BTArq::getRepeatedButton() {
	if (_lastButton == _button) return _lastButton;
	return BT_NO_BTN;
}

int BTArq::getStatus(e_BT_push_button ST_Button) const {
	return vDigitalMemoryRead(ST_Button);
}


