#include "BTArq.h"

float *V;           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
float *DV;           // This array is synchronized with Virtuino DV memory.
uint8_t v_button;

uint8_t _max_V;
uint8_t _max_DV;
uint8_t _BTN_V;


BTArq::BTArq(uint8_t max_V, uint8_t max_DV, uint8_t BTN_V){
	_max_V = max_V;
	_max_DV = max_DV;
	_BTN_V = BTN_V;
	V = new float [_max_V];
	for (int a=0; a<_max_V;a++) {
		V[a] = 0;
	}

	DV = new float [_max_DV];
	for (int a=0; a<_max_DV;a++) {
		DV[a] = 0;
	}

	v_button = BT_NO_BTN;

	_V= V;
	_DV= DV;

	};

BTArq::~BTArq() {};


// Return last pressed button and empty buffer
uint8_t BTArq::getButtonPressed()  {
	_lastButton = v_button;
	v_button = BT_NO_BTN;
	return _lastButton;

}


//============================================================== onCommandReceived
//==============================================================
/* This function is called every time Virtuino app sends a request to server to change a Pin value
 * The 'variableType' can be a character like V, D  V=Virtual pin  D = Virtual Digital pin (button)
 * The 'variableIndex' is the pin number index of Virtuino app
 * The 'valueAsText' is the value that has sent from the app   */
 void onReceived(char variableType, uint8_t variableIndex, String valueAsText){

	 if (variableType=='D' and variableIndex<_max_DV) {
		if (valueAsText == "1") {
			v_button = variableIndex;			// only update value of button at press time. v_button is set to 0 after execution of related action.
			DV[v_button]=1;              // copy the received value to arduino DV memory array
		} else if (valueAsText == "0") DV[variableIndex]=0;              // copy the received value to arduino DV memory array

	 } else if (variableType=='V' and variableIndex>= _BTN_V) {
			if (valueAsText == "1") {
				v_button = variableIndex- _BTN_V;			// only update value of button at press time. v_button is set to 0 after execution of related action.
				DV[v_button]=1;              // copy the received value to arduino DV memory array
			  	//DEBUG_print( "Button pressed\n");
		    } else if (valueAsText == "0") DV[variableIndex- _BTN_V]=0;              // copy the received value to arduino DV memory array

	 } else if (variableType=='V' and variableIndex<_max_V){
			float fvalue = valueAsText.toFloat();        // convert the value to float. The valueAsText has to be numerical
			V[variableIndex]=fvalue;              // copy the received value to arduino V memory array
     }
}

//==============================================================
/* This function is called every time Virtuino app requests to read a pin value*/
String onRequested(char variableType, uint8_t variableIndex){

    if (variableType=='V' and variableIndex<_max_V) {
    	return  String(V[variableIndex]);   // return the value of the arduino V memory array
  } else if (variableType=='D' and variableIndex<_max_DV) {
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


