/* Virtuino bluetooth library ver 1.61
 * Created by Ilias Lamprou
 * Updated AUG 1 2017
 * 
 * Download latest Virtuino android app from the link: https://play.google.com/store/apps/details?id=com.virtuino_automations.virtuino
 * Video tutorial link: https://www.youtube.com/watch?v=CYR_jigRkgk
 * Contact address for questions or comments: iliaslampr@gmail.com
 */

/*========= VirtuinoBluetooth Class methods  
*  vPinMode(int pin, int state)                                  set pin as digital OUTPUT or digital INPUT.  (Insteed default pinMode method
*
*========= Virtuino General methods  
*  void vDigitalMemoryWrite(int digitalMemoryIndex, int value)   write a value to a Virtuino digital memory   (digitalMemoryIndex=0..31, value range = 0 or 1)
*  int  vDigitalMemoryRead(int digitalMemoryIndex)               read  the value of a Virtuino digital memory (digitalMemoryIndex=0..31, returned value range = 0 or 1)
*  void vMemoryWrite(int analogMemoryIndex, float value)         write a value to Virtuino float memory       (memoryIndex=0..31, value range as float value)
*  float vMemoryRead(int analogMemoryIndex)                      read the value of  Virtuino analog memory    (analogMemoryIndex=0..31, returned value range = 0..1023)
*  run()                                                         neccesary command to communicate with Virtuino android app  (on start of void loop)
*  int getPinValue(int pin)                                      read the value of a Pin. Usefull for PWM pins
*  void clearTextBuffer();                                       Clear the text received text buffer
*  int textAvailable();                                          Check if there is text in the received buffer
*  String getText(byte ID);                                      Read the text from Virtuino app
*  void sendText(byte ID, String text);                          Send text to Virtuino app  
*/



#ifndef VirtuinoBluetooth_h
#define VirtuinoBluetooth_h

//#define BLUETOOTH_USE_SOFTWARE_SERIAL  // disable this line if you want to use hardware serial
// Modified SPM
#define BLUETOOTH_USE_NEO_HW_SERIAL // disable this line if you want to use hardware serial or software serial

#if defined(BLUETOOTH_USE_SOFTWARE_SERIAL) && defined(BLUETOOTH_USE_NEO_HW_SERIAL)
	#error Only one Bluetooth Serial should be defined in VirtuinoBluetooth.h!
#endif
// SPM end modif.


#include "Arduino.h"
#ifdef BLUETOOTH_USE_SOFTWARE_SERIAL
#include "SoftwareSerial.h"
// SPM Modified
#define BT_PORT_NAME "SoftwareSerial"
  #elif defined (BLUETOOTH_USE_NEO_HW_SERIAL)
#include "NeoHWSerial.h"
#define BT_PORT_NAME "NeoHWSerial"
// SPM end modif.
#endif

//=================   Board configuration ======= 

//#define ESP_GENERIC            // Enable this line to use this library with ESP8266 boards like NodeMCU, WEMOS etc.
 

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) 
      #define arduinoAnalogPinsSize 16  
      #define analogInputPinsMap_ { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15 }
      #define bt_arduinoPinsSize 54               // arduino mega board pins count 

#elif defined(ARDUINO_AVR_NANO)
      #define arduinoAnalogPinsSize 8 
      #define analogInputPinsMap_ { A0, A1, A2, A3, A4, A5, A6, A7}
      #define bt_arduinoPinsSize 14               // arduino nano digital pins 
      
#elif defined (__arm__) && defined (__SAM3X8E__)
      #define arduinoAnalogPinsSize 12 
      #define analogInputPinsMap_ { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11}
      #define bt_arduinoPinsSize 54               // arduino due digital pins 

#elif defined(ESP_GENERIC)
      #define arduinoAnalogPinsSize 1 
      #define analogInputPinsMap_ {A0}
      #define bt_arduinoPinsSize 11               //  esp board digital pins 
#else
      #define arduinoAnalogPinsSize 6           // default board
      #define analogInputPinsMap_ { A0, A1, A2, A3, A4, A5}
      #define bt_arduinoPinsSize 14               // arduino uno board pins count 
     
#endif

 
#define bt_virtualDigitalMemorySize  32     // DV virtual memory size 
#define bt_virtualAnalogMemorySize   32     //  V virtual memory size



#define bt_ERROR_PIN       1
#define bt_ERROR_VALUE     2
#define bt_ERROR_TYPE      3
#define bt_ERROR_SIZE      4
//#define bt_ERROR_PASSWORD  5
#define bt_ERROR_COMMAND   6
#define bt_ERROR_UNKNOWN   7

#define bt_COMMAND_START_CHAR '!'
#define bt_COMMAND_END_CHAR   '$'
#define bt_COMMAND_ERROR       "E00="

#define bt_firmwareCode "!C00=1.63$"                 
#define bt_TEXT_COMMAND_MAX_SIZE 200 




 class VirtuinoBluetooth {
  public:

  #ifdef BLUETOOTH_USE_SOFTWARE_SERIAL
    VirtuinoBluetooth(SoftwareSerial &uart);
    VirtuinoBluetooth(SoftwareSerial &uart, uint32_t baud);
  // SPM Modified
  #elif defined (BLUETOOTH_USE_NEO_HW_SERIAL)
    	VirtuinoBluetooth(NeoHWSerial &uart);
    	VirtuinoBluetooth(NeoHWSerial &uart, uint32_t baud);
  // SPM end modif.
  #else
    VirtuinoBluetooth(HardwareSerial &uart);
    VirtuinoBluetooth(HardwareSerial &uart, uint32_t baud);
  #endif

 
  void run();
  void vDigitalMemoryWrite(int digitalMemoryIndex, int value);
  int vDigitalMemoryRead(int digitalMemoryIndex);
  void vMemoryWrite(int memoryIndex, float value);
  float vMemoryRead(int memoryIndex);
  int getPinValue(int pin);
  void vPinMode(int pin, int mode); 
  boolean DEBUG=false;
  long lastCommunicationTime;
  void vDelay(long milliseconds);

  //-- Text Command functions
  void clearTextBuffer();
  int textAvailable();
  String getText(byte ID);
  void sendText(byte ID, String text);
  String btResponseBuffer="";
  
  
  private:
  void checkIfIOsHaveChanged();
  void checkVirtuinoCommand(String* command);
  void sendCommandResponse(char commandType, int commandPin , String commandValueString);
  void  readBluetoothSerialData();
  void executeReceivedCommand(char activeCommandType, int activeCommandPin ,String* commandString, boolean returnInfo);
  String getErrorCommand(byte code);
  char  getCommandType(char c);
  int  getCommandPin(String* aCommand);
  float getCommandValue(String* aCommand);
  
  int arduinoPinsMap[bt_arduinoPinsSize] ;
  int arduinoPinsValue[bt_arduinoPinsSize];
  int virtualDigitalMemory[bt_virtualDigitalMemorySize];                       
  int virtualDigitalMemoryIdol[bt_virtualDigitalMemorySize]; 
  float virtualFloatMemory[bt_virtualAnalogMemorySize] ;                         
  int analogInputPinsMap[arduinoAnalogPinsSize]  = analogInputPinsMap_  ;
  String commandBuffer="";                                         // Store the active command received from app
 
  boolean newCommand=false;                                        // This flag takes the value 1 every time we have a commplete command from app
  bool  connectedStatus=true;

  //-- Text Command functions
  String textReceivedCommandBuffer="";
  String textToSendCommandBuffer="";
  void urlencode(String* str);
  void urldecode(String* str);
  void clearTextByID(byte ID, String* textBuffer);
  void addTextToReceivedBuffer(byte ID, String* text);

  protected: //SPM MODIF: Declared protected to be used by BTArq in AT mode
  
  #ifdef BLUETOOTH_USE_SOFTWARE_SERIAL
      SoftwareSerial *BTserial;
  // SPM Modified
  #elif defined (BLUETOOTH_USE_NEO_HW_SERIAL)
      NeoHWSerial *BTserial;
  // SPM end modif
  #else
      HardwareSerial *BTserial;
  #endif                                                                 

 };



 
#endif


