#include "BTArq.h"


// SPM INI VIRTUINOCM
//// SPM original:
//// BTArq::BTArq(): VirtuinoBluetooth(Serial1)
//// SPM modified
//BTArq::BTArq()
//: VirtuinoBluetooth(BTPort)
//{
//	// Code to use HardwareSerial
//	// VirtuinoBluetooth virtuino(Serial1);            // enable this line and disable all SoftwareSerial lines
//    // Open VirtuinoBluetooth.h file on the virtuino library folder ->  disable the line: #define BLUETOOTH_USE_SOFTWARE_SERIAL
//}


//================================================================== VirtuinoBluetooth init
#ifdef BLUETOOTH_USE_SOFTWARE_SERIAL
BTArq::BTArq(SoftwareSerial &uart, uint32_t baud): BTserial(&uart){
    BTserial->begin(baud);
    while (BTserial->available()) BTserial->read();
}

BTArq::BTArq(SoftwareSerial &uart): BTserial(&uart){}

// SPM Modified
#elif defined (BLUETOOTH_USE_NEO_HW_SERIAL)
BTArq::BTArq(NeoHWSerial &uart, uint32_t baud): BTserial(&uart){

	BTserial->begin(baud);
    while (BTserial->available()) BTserial->read();

}
BTArq::BTArq(NeoHWSerial &uart): BTserial(&uart){



}
// SPM end modif
#else
BTArq::BTArq(HardwareSerial &uart, uint32_t baud): BTserial(&uart)
: VirtuinoCM()
{

	BTserial->begin(baud);
    while (BTserial->available()) BTserial->read();
}
BTArq::BTArq(HardwareSerial &uart): BTserial(&uart){}

#endif

// SPM FIN VIRTUINOCM


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

	//SPM INI VIRTUINOCM
	//run();            //  necessary command to communicate with Virtuino android app

	virtuinoRun();        // Necessary function to communicate with Virtuino. Client handler

	  // enter your code below. Avoid to use delays on this loop. Instead of the default delay function use the vDelay that is located on the bottom of this code
	  // You don't need to add code to read or write to the pins. Just enter the  pinMode of each Pin you want to use on void setup
	//SPM INI VIRTUINOCM


	//SPM INI VIRTUINOCM-->COMENTADO PARA TEST
//	// EVALUATE PUSH BUTTONS STATUS IN APP
//	int i= int (START_BT);
//	while (++i != int(MAX_BT)) {
//		if (vDigitalMemoryRead(i)==HIGH) {
//			button = e_BT_push_button(i);
//			reset_button=true;
//		}
//	}
//	setButton(button);
//
//	// UPDATE TO APP:
//	//Reset buttons if necessary
//	if (reset_button==true){
//		i= int (START_BT);
//		while (++i != int(MAX_BT)) {
//			vDigitalMemoryWrite(i, LOW);
//		}
//	}
//
//	//Write FLOAT VPIN
//	i= int (START_AI);
//	while (++i != int(MAX_AI)) {
//		vMemoryWrite(i, FM[i]);
//	}
//
//
}


// Return last pressed button and empty buffer
// Repeated button are informed
e_BT_push_button BTArq::getButtonPressed()  {
//	if (_lastButton == _button) return BT_BTN_REPEATED;
//	_lastButton = _button;
//	_button = BT_NO_BTN;
//	return _lastButton;
}

void BTArq::setButton(e_BT_push_button button) {
//	_button = button;
}

// return button when button is repeated
e_BT_push_button BTArq::getRepeatedButton() {
//	if (_lastButton == _button) return _lastButton;
//	return BT_NO_BTN;
}

int BTArq::getStatus(e_BT_push_button ST_Button) const {
//	return vDigitalMemoryRead(ST_Button);
}
//SPM FIN VIRTUINOCM-->COMENTADO PARA TEST

//SPM INI VIRTUINOCM

//============================================================== onCommandReceived
//==============================================================
/* This function is called every time Virtuino app sends a request to server to change a Pin value
 * The 'variableType' can be a character like V, T, O  V=Virtual pin  T=Text Pin    O=PWM Pin
 * The 'variableIndex' is the pin number index of Virtuino app
 * The 'valueAsText' is the value that has sent from the app   */
 void BTArq::onReceived(char variableType, uint8_t variableIndex, String valueAsText){
    if (variableType=='V'){
        float value = valueAsText.toFloat();        // convert the value to float. The valueAsText have to be numerical
        if (variableIndex<V_memory_count) V[variableIndex]=value;              // copy the received value to arduino V memory array
    }
}

//==============================================================
/* This function is called every time Virtuino app requests to read a pin value*/
String BTArq::onRequested(char variableType, uint8_t variableIndex){
    if (variableType=='V') {
    if (variableIndex<V_memory_count) return  String(V[variableIndex]);   // return the value of the arduino V memory array
  }
  return "";
}

//============================================================== virtuinoRun
  void BTArq::virtuinoRun(){
    while (BTserial->available()) {
        char tempChar=BTserial->read();
        if (tempChar==CM_START_CHAR) {               // a new command is starting...
              readBuffer=CM_START_CHAR;     // copy the new command to the virtuino readBuffer
              readBuffer+=BTserial->readStringUntil(CM_END_CHAR);
              readBuffer+=CM_END_CHAR;
              if (debug) NeoSerial.println("\nCommand= "+readBuffer);
              String* response= getResponse();    // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
              if (debug) NeoSerial.println("Response : "+*response);
              BTserial->print(*response);
              break;
         }
    }
 }


 //============================================================== vDelay
  void BTArq::vDelay(int delayInMillis){long t=millis()+delayInMillis;while (millis()<t) virtuinoRun();}

  //SPM FIN VIRTUINOCM
