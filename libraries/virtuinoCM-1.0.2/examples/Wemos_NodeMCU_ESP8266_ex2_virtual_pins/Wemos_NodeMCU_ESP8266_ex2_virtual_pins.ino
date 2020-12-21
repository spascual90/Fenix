/* Example: Wemos - NodeMCU ESP8266 - Virtual Pins V 
 * Created by Ilias Lamprou
 * Modified: Sep/9/2019
 */

#include <ESP8266WiFi.h>

//--- SETTINGS ------------------------------------------------
const char* ssid = "WIFI NETWORK";        // WIFI network SSID
const char* password = "PASSWORD";        // WIFI network PASSWORD
WiFiServer server(8000);                   // Default Virtuino Server port 
IPAddress ip(192, 168, 1, 150);            // where 150 is the desired IP Address. The first three numbers must be the same as the router IP
IPAddress gateway(192, 168, 1, 1);         // set gateway to match your network. Replace with your router IP
//---

//---VirtuinoCM  Library settings --------------
#include "VirtuinoCM.h"
VirtuinoCM virtuino;               
#define V_memory_count 32          // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
//---


boolean debug = true;              // set this variable to false on the finale code to decrease the request time.

float V1_lastValue=0;

//============================================================== setup
//==============================================================
void setup() {
  if (debug) {
    Serial.begin(9600);
    while (!Serial) continue;
  }
 
  virtuino.begin(onReceived,onRequested,256);  //Start Virtuino. Set the buffer to 256. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes
  //virtuino.key="1234";                       //This is the Virtuino password. Only requests the start with this key are accepted from the library

  connectToWiFiNetwork();
  server.begin();
  
  pinMode(D4, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(D5, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(D6, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(D7, INPUT);             // On Virtuino panel add a led to get the state of this pin
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

    //--- Example2:  How to read the value of V2 pin from Virtuino. How to control two digital pins using a virtual pin
    if (V[2]==1) {
      digitalWrite(D5,HIGH);              // On Virtuino add a button to control the pin V2
      digitalWrite(D6,HIGH); 
    }
    else {
      digitalWrite(D5,LOW);
      digitalWrite(D6,LOW); 
    }
  
    
   //---Example3: How to write sensor values to the virtual pins V4 & V5
   int sensorValue= random(100);   // This is the value that you want to send to virtuino. Replace with your sensor value
   V[4] = sensorValue;             // copy the sensor value to the pin V4. That's all!  On Virtuino panel add two value displays to get these values
   V[5] = V[4]*10;
  
  vDelay(1000);     // This is an example of the recommended delay function. Remove this if you don't need
}







//============================================================== connectToWiFiNetwork
void connectToWiFiNetwork(){
  Serial.println("Connecting to "+String(ssid));
   // If you don't want to config IP manually disable the next two lines
   IPAddress subnet(255, 255, 255, 0);        // set subnet mask to match your network
  WiFi.config(ip, gateway, subnet);          // If you don't want to config IP manually disable this line
  WiFi.mode(WIFI_STA);                       // Config module as station only.
  WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
    }
   Serial.println("");
   Serial.println("WiFi connected");
   Serial.println(WiFi.localIP());
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
