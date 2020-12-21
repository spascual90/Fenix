/* Example: Wemos - NodeMCU ESP8266 - Read-Write IOs only (This example doesn't support Virtual pins)
 * Created by Ilias Lamprou
 * Modified: Sep/9/2019
 */

#include <ESP8266WiFi.h>

//--- SETTINGS ------------------------------------------------
const char* ssid = "WIFI NETWORK";        // WIFI network SSID
const char* password = "PASSWORD";       // WIFI network PASSWORD
WiFiServer server(8000);                   // Default Virtuino Server port 
IPAddress ip(192, 168, 1, 150);            // 150 is the desired IP Address. The first three numbers must be the same as the router IP
IPAddress gateway(192, 168, 1, 1);         // set gateway to match your network. Replace it with your router IP
//---

//---VirtuinoCM  Library settings --------------
#include "VirtuinoCM.h"
VirtuinoCM virtuino;               

boolean debug = true;              // set this variable to false on the finale code to decrease the request time.
//============================================================== setup
//==============================================================
void setup() {
  if (debug) {
    Serial.begin(9600);
    while (!Serial) continue;
  }
 
  virtuino.begin(NULL,NULL,256);  //Start Virtuino. Set the buffer to 256. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes
  //virtuino.key="1234";                       //This is the Virtuino password. Only requests the start with this key are accepted from the library

  connectToWiFiNetwork();
  server.begin();
  
  pinMode(D0, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(D1, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(D2, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(D3, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(D4, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(D5, INPUT);             // On Virtuino panel add a led to get the state of this pin
  pinMode(D6, INPUT);             // On Virtuino panel add a led to get the state of this pin
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
