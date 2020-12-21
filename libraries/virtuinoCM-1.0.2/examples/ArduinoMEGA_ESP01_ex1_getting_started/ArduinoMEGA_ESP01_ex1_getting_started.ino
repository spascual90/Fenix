/* Example: HC-05 + Arduino MEGA (Hardware Serial Connection) - Getting started
 * Created by Ilias Lamprou
 * Modified: Sep/9/2019
 */


//--- SETTINGS ------------------------------------------------
const char* ssid = "WIFI NETWORK";        // WIFI network SSID
const char* password = "PASSWORD";       // WIFI network PASSWORD
int port=8000;                             // Virtuino default Server port
const char* serverIP = "192.168.1.150";    // The three first numbers have to be the same with the router IP
//-------------------------------------------------------------

//-------------VirtuinoCM  Library and settings --------------
#include "VirtuinoCM.h"
VirtuinoCM virtuino;               
#define V_memory_count 32          // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.

boolean debug = true;              // set this variable to false on the finale code to decrease the request time.


//============================================================== setup
//==============================================================
void setup() {
  if (debug) {
    Serial.begin(9600);
    while (!Serial) continue;
  }
  
  Serial1.begin(9600);     // or 115200
  Serial1.setTimeout(50);

  virtuino.begin(onReceived,onRequested,256);  //Start Virtuino. Set the buffer to 256. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes
  //virtuino.key="1234";                       //This is the Virtuino password. Only requests the start with this key are accepted from the library

  connectToWiFiNetwork();
   
  pinMode(4, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(13, OUTPUT);            // On Virtuino panel add a button to control this pin
  pinMode(6, OUTPUT);             
  pinMode(7, INPUT);             // On Virtuino panel add a led to get the state of this pin
  }

//============================================================== loop
//==============================================================
void loop() {
  virtuinoRun();        // Necessary function to communicate with Virtuino. Client handler

  // enter your code below. Avoid to use delays on this loop. Instead of the default delay function use the vDelay that is located on the bottom of this code
  // You don't need to add code to read or write to the pins. Just enter the  pinMode of each Pin you want to use on void setup

  
  
  vDelay(1000);     // This is an example of the recommended delay function. Remove this if you don't need
}




//================================================ connectToWiFiNetwork
void connectToWiFiNetwork(){
    Serial.println("Connecting to "+String(ssid));
    while (Serial1.available()) Serial1.read();
    Serial1.println("AT+GMR");       // print firmware info
    waitForResponse("OK",1000);
    Serial1.println("AT+CWMODE=1");  // configure as client
    waitForResponse("OK",1000);
    Serial1.print("AT+CWJAP=\"");    // connect to your WiFi network
    Serial1.print(ssid);
    Serial1.print("\",\"");
    Serial1.print(password);
    Serial1.println("\"");
    waitForResponse("OK",10000);
    Serial1.print("AT+CIPSTA=\"");   // set IP
    Serial1.print(serverIP);
    Serial1.println("\"");   
    waitForResponse("OK",5000);
    Serial1.println("AT+CIPSTA?");
    waitForResponse("OK",3000); 
    Serial1.println("AT+CIFSR");           // get ip address
    waitForResponse("OK",1000);
    Serial1.println("AT+CIPMUX=1");         // configure for multiple connections   
    waitForResponse("OK",1000);
    Serial1.print("AT+CIPSERVER=1,");
    Serial1.println(port);
    waitForResponse("OK",1000);
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
  if(Serial1.available()){
        virtuino.readBuffer = Serial1.readStringUntil('\n');
        if (debug) Serial.print('\n'+virtuino.readBuffer);
        int pos=virtuino.readBuffer.indexOf("+IPD,");
        if (pos!=-1){
              int connectionId = virtuino.readBuffer.charAt(pos+5)-48;  // get connection ID
              int startVirtuinoCommandPos = 1+virtuino.readBuffer.indexOf(":");
              virtuino.readBuffer.remove(0,startVirtuinoCommandPos);
              String* response= virtuino.getResponse();    // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
              if (debug) Serial.println("\nResponse : "+*response);
              if (response->length()>0) {
                String cipSend = "AT+CIPSEND=";
                cipSend += connectionId;
                cipSend += ",";
                cipSend += response->length();
                cipSend += "\r\n";
                while(Serial1.available()) Serial1.read();    // clear Serial1 buffer 
                for (int i=0;i<cipSend.length();i++) Serial1.write(cipSend.charAt(i));
                if (waitForResponse(">",1000)) Serial1.print(*response);
                waitForResponse("OK",1000);
              }
              Serial1.print("AT+CIPCLOSE=");Serial1.println(connectionId);
         }// (pos!=-1)
           
  } // if Serial1.available
        
}

 
//=================================================== waitForResponse
boolean waitForResponse(String target1,  int timeout){
    String data="";
    char a;
    unsigned long startTime = millis();
    boolean rValue=false;
    while (millis() - startTime < timeout) {
        while(Serial1.available() > 0) {
            a = Serial1.read();
            if (debug) Serial.print(a);
            if(a == '\0') continue;
            data += a;
        }
        if (data.indexOf(target1) != -1) {
            rValue=true;
            break;
        } 
    }
    return rValue;
}


 //============================================================== vDelay
  void vDelay(int delayInMillis){long t=millis()+delayInMillis;while (millis()<t) virtuinoRun();}
