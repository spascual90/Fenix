/* Example: Wemos - NodeMCU ESP8266 - Access Point 
 * Created by Ilias Lamprou
 * Modified: Sep/9/2019
 */

#include <ESP8266WiFi.h>

//--- SETTINGS ------------------------------------------------
const char* ssid_AP = "WIFI NETWORK";   // the SSID for the ESP8266 network
const char* password_AP = "PASSWORD";    // the password for ESP8266 network 
WiFiServer server(8000);                   // Default Virtuino Server port 
//---

//---VirtuinoCM  Library settings --------------
#include "VirtuinoCM.h"
VirtuinoCM virtuino;               
#define V_memory_count 32          // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
//---


boolean debug = true;              // set this variable to false on the finale code to decrease the request time.

//============================================================== setup
//==============================================================
void setup() {
  if (debug) {
    Serial.begin(9600);
    while (!Serial) continue;
  }
 
  virtuino.begin(onReceived,onRequested,256);  //Start Virtuino. Set the buffer to 256. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes
  //virtuino.key="1234";                       //This is the Virtuino password. Only requests the start with this key are accepted from the library

   initAccessPoint();
  server.begin();
  
  pinMode(D4, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(D5, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(D6, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(D7, INPUT);             // On Virtuino panel add a led to get the state of this pin
  pinMode(D8, INPUT);             // On Virtuino panel add a led to get the state of this pin
  }

//============================================================== loop
//==============================================================
void loop() {
  virtuinoRun();        // Necessary function to communicate with Virtuino. Client handler

  // enter your code below. Avoid to use delays on this loop. Instead of the default delay function use the vDelay that is located on the bottom of this code
  // You don't need to add code to read or write to the pins. Just enter the  pinMode of each Pin you want to use on void setup

  
  vDelay(1000);     // This is an example of the recommended delay function. Remove this if you don't need
}







//================================================================= initAccessPoint
void initAccessPoint(){
   if (debug) Serial.print("Access Point");                   // Default IP: 192.168.4.1
   WiFi.mode(WIFI_AP);                                     // Config module as Access point only.  Set WiFi.mode(WIFI_AP_STA); to config module as Acces point and station
   boolean result = WiFi.softAP(ssid_AP,password_AP);      // set the Access point SSID and password
   if(result == true)  {
     if (debug)Serial.println("Server Ready");
     if (debug) Serial.println(WiFi.softAPIP());
   }
   else  if (debug)Serial.println("Failed!");
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


 //==============================================================
  void virtuinoRun(){
   WiFiClient client = server.available();
   if (!client) return;
   if (debug) Serial.println("Connected");
   unsigned long timeout = millis() + 3000;
   while (!client.available() && millis() < timeout) delay(1);
   if (millis() > timeout) {
    Serial.println("timeout");
    client.flush();
    client.stop();
    return;
  }
    virtuino.readBuffer="";    // clear Virtuino input buffer. The inputBuffer stores the incoming characters
      while (client.available()>0) {        
        char c = client.read();         // read the incoming data
        virtuino.readBuffer+=c;         // add the incoming character to Virtuino input buffer
        if (debug) Serial.write(c);
      }
     client.flush();
     if (debug) Serial.println("\nReceived data: "+virtuino.readBuffer);
     String* response= virtuino.getResponse();    // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
     if (debug) Serial.println("Response : "+*response);
     client.print(*response);
     client.flush();
     delay(10);
     client.stop(); 
    if (debug) Serial.println("Disconnected");
}


 //============================================================== vDelay
  void vDelay(int delayInMillis){long t=millis()+delayInMillis;while (millis()<t) virtuinoRun();}
