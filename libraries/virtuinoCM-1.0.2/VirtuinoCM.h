/* VirtuinoCM library ver 1.0.02
 * Created by Ilias Lamprou
 * Updated Sep 10 2019
 * Download latest Virtuino android app from the link: https://play.google.com/store/apps/details?id=com.virtuino_automations.virtuino
 */

#ifndef VirtuinoCM_h
#define VirtuinoCM_h

#include "Arduino.h"

#define CM_WELCOME_MESSAGE  "!C=VirtuinoCM 1.0.02" 
#define CM_START_CHAR '!'                 // All Virtuino commands starts with !
#define CM_END_CHAR   '$'                 // All Virtuino commands ends with $
#define CM_ERROR  "E00=7$"           
#define CM_ERROR_KEY "E00=5$"           
//====================================================================
 class VirtuinoCM {
  public:                                            
  VirtuinoCM();
  void begin(void (*callback1)(char,uint8_t,String),String (*callback2)(char,uint8_t),size_t bufSize);
  String*  getResponse();     
  String key=""; 
  boolean manualPWMcontrol=false;
  String readBuffer="";
  String writeBuffer="";

  private:
  size_t bufSize = 256;                   // default readBuffer and writeBuffer size
  const char *NO_REPLY= "_NR_";
  String urlencode(String* str);
  unsigned char h2int(char c);
  String urldecode(String* str);
  void (*receivedHandler)(char,uint8_t,String);
  String (*requestedHandler)(char,uint8_t); 
 };


#endif
