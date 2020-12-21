/* Example: Bluetooth (HC-05 or (HC-06) + Arduino UNO - Virtual Pins V 
 * Created by Ilias Lamprou
 * Modified: Sep/9/2019
 */

#include <SoftwareSerial.h>
SoftwareSerial espSerial =  SoftwareSerial(2,3);      // arduino RX pin=2  arduino TX pin=3    connect the arduino RX pin to esp8266 module TX pin   -  connect the arduino TX pin to esp8266 module RX pin

//-------------VirtuinoCM  Library and settings --------------
#include "VirtuinoCM.h"
VirtuinoCM virtuino;               
#define V_memory_count 32          // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.

boolean debug = true;              // set this variable to false on the finale code to decrease the request time.

float V1_lastValue=0;
int counterDemo=0;
//============================================================== setup
//==============================================================
void setup() {
  if (debug) {
    Serial.begin(9600);
    while (!Serial) continue;
  }
  espSerial.begin(9600);  
  espSerial.setTimeout(50);

  virtuino.begin(onReceived,onRequested,256);  //Start Virtuino. Set the buffer to 256. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes
  //virtuino.key="1234";                       //This is the Virtuino password. Only requests the start with this key are accepted from the library
  
  pinMode(4, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(6, OUTPUT);            
  pinMode(13, OUTPUT);           // On Virtuino panel add a button to control this pin
  pinMode(7, INPUT);             // On Virtuino panel add a led to get the state of this pin
  }

//============================================================== loop
//==============================================================
void loop() {
  virtuinoRun();        // Necessary function to communicate with Virtuino. Client handler

  // enter your code below. Avoid to use delays on this loop. Instead of the default delay function use the vDelay that is located on the bottom of this code
  // You don't need to add code to read or write to the pins. Just enter the  pinMode of each Pin you want to use on void setup



 
  //--- Example1:  How to read the value of V1 pin from Virtuino. On Virtuino add a slider. Select the pin V1 
    if (V[1]!=V1_lastValue) {              // The V1 has changed
      Serial.println("V1="+String(V[1]));  // print the value of V1
      V1_lastValue=V[1];                   // store the V1 to the variable V1_lastValue so as to know if it has changed
    }
    
  //---Example2:  How to read the value of V2 pin from Virtuino.  On Virtuino add a slider. Select the pin V1. Set the range to (0,2)
    if (V[2]==0) analogWrite(6,0);
    else if (V[2]==1) analogWrite(6,128);
    else if (V[2]==2) analogWrite(6,255);
  
    
   //---Example3: How to write a sensor value to the virtual pin V4
   int sensorValue= random(100);   // This is the value that you want to send to virtuino. Replace with your sensor value
   V[4] = sensorValue;             // copy the sensor value to the pin V4. That's all!  On Virtuino panel add a value display to get this value

   //---Example4 How to write a counter value to the virtual pin V5
    V[5] = counterDemo;             // copy the sensor value to the pin V5. That's all!  On Virtuino panel add a value display to get this value
    counterDemo++;
    if (counterDemo==100) counterDemo=0;   // restart the counter

  
  vDelay(1000);     // This is an example of the recommended delay function. Remove this if you don't need
}









//============================================================== onCommandReceived
//==============================================================
/* This function is called every time Virtuino app sends a request to server to change a Pin value
 * The 'variableType' can be a character like V, T, O  V=Virtual pin  T=Text Pin    O=PWM Pin 
 * The 'variableIndex' is the pin number index of Virtuino app
 * The 'valueAsText' is the value that has sent from the app   */
 void onReceived(char variableType, uint8_t variableIndex, String valueAsText){     
    if (variableType=='V'){
        float value = valueAsText.toFloat();        // convert the value to float. The valueAsText have to be numerical
        if (variableIndex<V_memory_count) V[variableIndex]=value;              // copy the received value to arduino V memory array
    }
}

//==============================================================
/* This function is called every time Virtuino app requests to read a pin value*/
String onRequested(char variableType, uint8_t variableIndex){     
    if (variableType=='V') {
    if (variableIndex<V_memory_count) return  String(V[variableIndex]);   // return the value of the arduino V memory array
  }
  return "";
}

//============================================================== virtuinoRun
  void virtuinoRun(){
    while (espSerial.available()) {
        char tempChar=espSerial.read();
        if (tempChar==CM_START_CHAR) {               // a new command is starting...
              virtuino.readBuffer=CM_START_CHAR;     // copy the new command to the virtuino readBuffer
              virtuino.readBuffer+=espSerial.readStringUntil(CM_END_CHAR);
              virtuino.readBuffer+=CM_END_CHAR;
              if (debug) Serial.println("\nCommand= "+virtuino.readBuffer);
              String* response= virtuino.getResponse();    // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
              if (debug) Serial.println("Response : "+*response);
              espSerial.print(*response);
              break; 
         }
    }
 }
 

 //============================================================== vDelay
  void vDelay(int delayInMillis){long t=millis()+delayInMillis;while (millis()<t) virtuinoRun();}
