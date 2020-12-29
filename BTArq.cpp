#include "BTArq.h"

//int variable_prueba;

float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
float DV[DV_memory_count];           // This array is synchronized with Virtuino DV memory.
e_BT_push_button v_button;

BTArq::BTArq(){
for (int a=0; a<V_memory_count;a++) {
	V[a] = 0;
}
for (int a=0; a<DV_memory_count;a++) {
	DV[a] = 0;
}
e_BT_push_button v_button = BT_NO_BTN;
};

BTArq::~BTArq() {};

void BTArq::updateBT () {
	virtuinoRun();        // Necessary function to communicate with Virtuino. Client handler
}


//// Return last pressed button and empty buffer
e_BT_push_button BTArq::getButtonPressed()  {
	_lastButton = v_button;
	v_button = BT_NO_BTN;
	return _lastButton;

}


//============================================================== onCommandReceived
//==============================================================
/* This function is called every time Virtuino app sends a request to server to change a Pin value
 * The 'variableType' can be a character like V, T, O  V=Virtual pin  T=Text Pin    O=PWM Pin
 * The 'variableIndex' is the pin number index of Virtuino app
 * The 'valueAsText' is the value that has sent from the app   */
 void onReceived(char variableType, uint8_t variableIndex, String valueAsText){

	 if (variableType=='D') {
		e_BT_push_button button = (e_BT_push_button) variableIndex;
		if (button>START_BT && button<MAX_BT) {
			if (valueAsText == "1") {
				v_button = (e_BT_push_button)variableIndex;
			}
		}
	} else if (variableType=='V'){
        float fvalue = valueAsText.toFloat();        // convert the value to float. The valueAsText have to be numerical
        if (variableIndex<V_memory_count) {
        	V[variableIndex]=fvalue;              // copy the received value to arduino V memory array
        }
    }
}

//==============================================================
/* This function is called every time Virtuino app requests to read a pin value*/
String onRequested(char variableType, uint8_t variableIndex){

    if (variableType=='V' and variableIndex<V_memory_count) {
    	return  String(V[variableIndex]);   // return the value of the arduino V memory array
  } else if (variableType=='D' and variableIndex<DV_memory_count) {
	  return  String(DV[variableIndex]); // return the value of the arduino DV memory array
  }

  return "";
}

//============================================================== virtuinoRun
  void BTArq::virtuinoRun(){
	while (BTPort.available()) {
        char tempChar=BTPort.read();
        if (tempChar==CM_START_CHAR) {      // a new command is starting...
              readBuffer=CM_START_CHAR;     // copy the new command to the virtuino readBuffer
              readBuffer+=BTPort.readStringUntil(CM_END_CHAR);
              readBuffer+=CM_END_CHAR;
              if (debug) NeoSerial.println("\nCommand= "+readBuffer);
              String* response= getResponse();    // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
              if (debug) NeoSerial.println("Response : "+*response);
              BTPort.print(*response);
              break;
         }
    }
 }

//============================================================== vDelay
  void BTArq::vDelay(int delayInMillis){long t=millis()+delayInMillis;while (millis()<t) virtuinoRun();}


