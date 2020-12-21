/* VirtuinoCM library ver 1.0.02
 * Created by Ilias Lamprou
 * Updated Sep 10 2019
 * Download latest Virtuino android app from the link: https://play.google.com/store/apps/details?id=com.virtuino_automations.virtuino
 */


#include "VirtuinoCM.h"
//===================================================== VirtuinoSE
 VirtuinoCM::VirtuinoCM(){}
 //===================================================== begin
  void VirtuinoCM::begin(void (*callback1)(char,uint8_t,String),String (*callback2)(char,uint8_t),size_t bufSize){
    receivedHandler=callback1;
    requestedHandler=callback2;
    readBuffer.reserve(bufSize);
    writeBuffer.reserve(bufSize);
  }
  
//===================================================== getResponse
// This function checks the incoming Virtuino request.
// Returns a String that has to sent to the app as reply
String* VirtuinoCM::getResponse(){
     writeBuffer="";
     int startPos=readBuffer.indexOf(CM_START_CHAR);
     if (startPos==-1) {
      writeBuffer=CM_ERROR;
      return &writeBuffer;
     }
     //-----------check  password
     if (key.length()>0) {
       String commandPassword=readBuffer.substring(0,startPos);
       if (!key.equals(commandPassword)) {
        writeBuffer=CM_ERROR_KEY;
        return &writeBuffer;
       }
     }
     //-----------check  incomming commands
     String response="";
     while (startPos!=-1){
              int lastPos=readBuffer.indexOf(CM_END_CHAR,startPos+1);
              if (lastPos-startPos>5){
                String oneCommand = readBuffer.substring(startPos+1,lastPos); // get one command like A02=? or V14=23.78
                char commandType = oneCommand.charAt(0);               // get the type of command A,V,Q,O,T,C     
                if (commandType=='C') {
                  writeBuffer=CM_WELCOME_MESSAGE;
                  return &writeBuffer;
                }
                int equalPos= oneCommand.indexOf('=');
                if (equalPos>1) {
                   String pinText = oneCommand.substring(1,equalPos);  // get the pin as text
                   int pin=pinText.toInt();                            // get the pin
                   String value= oneCommand.substring(equalPos+1);     // get the value as text
                   if (value=="?"){                                    //1.check if command is a request to a pin value
                    String commandResponse=NO_REPLY;
                    switch (commandType){
                     case 'T': if (requestedHandler!=NULL){
                                  commandResponse=requestedHandler('T',pin);
                                  if (commandResponse.length()>0) commandResponse=urlencode(&commandResponse); 
                                }
                                  break;
                     case 'V': if (requestedHandler!=NULL) commandResponse=requestedHandler('V',pin);
                               if (commandResponse.length()==0) commandResponse=NO_REPLY;
                               break;
                     case 'A': commandResponse="";
                               commandResponse+=analogRead(pin);
                               break;
                     case 'Q': commandResponse="";
                               commandResponse+=digitalRead(pin);
                               break;
                     case 'O': if (manualPWMcontrol) {
                                  if (requestedHandler!=NULL) commandResponse=requestedHandler('O',pin); // recommended to set the manualPWMcontrol to TRUE
                                }
                               else{   
                                 #ifndef ARDUINO_ARCH_ESP32   // PWM pins for are not supported
                                  int pwm_value = pulseIn(pin, HIGH);   // Avoid to control or read PWM pins immediately. 
                                  pwm_value= pwm_value /7.85;           // This will cause a delay
                                  commandResponse="";
                                  commandResponse+=pwm_value;         // 
                                #endif
                               }
                               break;
                    } //switch
                    if (!commandResponse.equals(NO_REPLY)){
                       writeBuffer+=CM_START_CHAR;
                       writeBuffer+=commandType;
                       writeBuffer+=pin;
                       writeBuffer+="=";
                       writeBuffer+=commandResponse;
                       writeBuffer+=CM_END_CHAR;
                    }
                   } //if (value=="?")
                   else {                           //1.check if command contains a new pin value
                     if (commandType=='V') {
                        if (receivedHandler!=NULL) receivedHandler(commandType,pin,value); 
                     }
                     else if (commandType=='T') {
                        if (receivedHandler!=NULL) receivedHandler(commandType,pin,urldecode(&value)); 
                     }
                     else if (commandType=='Q'){
                       if (value.toInt()==0) digitalWrite(pin,LOW); 
                       else  digitalWrite(pin,HIGH); 
                     }
                     else if (commandType=='O'){
                      if (manualPWMcontrol) {
                        if (receivedHandler!=NULL) receivedHandler(commandType,pin,value); 
                      }
                      else {
                         #ifndef ARDUINO_ARCH_ESP32   // PWM pins for are not supported
                            analogWrite(pin, value.toInt()); 
                        #endif
                        
                      }
                     }
                   }
                }// if (equalPos>1) {
              } // if (lastPos-startPos>5)
              startPos=readBuffer.indexOf(CM_START_CHAR,lastPos);
     } //while
   return &writeBuffer;
}


//================================================================================== urlencode
//==================================================================================
String VirtuinoCM::urlencode(String* str){
   String encodedString="";
    char c;
    char code0;
    char code1;
    char code2;
    for (int i =0; i < str->length(); i++){
      c=str->charAt(i);
      if (c == ' '){
        encodedString+= '+';
      } else if (isalnum(c)){
        encodedString+=c;
      } else{
        code1=(c & 0xf)+'0';
        if ((c & 0xf) >9){
            code1=(c & 0xf) - 10 + 'A';
        }
        c=(c>>4)&0xf;
        code0=c+'0';
        if (c > 9){
            code0=c - 10 + 'A';
        }
        code2='\0';
        encodedString+='%';
        encodedString+=code0;
        encodedString+=code1;
        //encodedString+=code2;
      }
      yield();
    }
    return encodedString;
    
}

//================================================================================== h2int
//==================================================================================
unsigned char VirtuinoCM::h2int(char c){
    if (c >= '0' && c <='9'){
        return((unsigned char)c - '0');
    }
    if (c >= 'a' && c <='f'){
        return((unsigned char)c - 'a' + 10);
    }
    if (c >= 'A' && c <='F'){
        return((unsigned char)c - 'A' + 10);
    }
    return(0);
}
//================================================================================== urldecode
//==================================================================================
String VirtuinoCM::urldecode(String* str){
   String encodedString="";
    char c;
    char code0;
    char code1;
    for (int i =0; i < str->length(); i++){
        c=str->charAt(i);
      if (c == '+'){
        encodedString+=' ';  
      }else if (c == '%') {
        i++;
        code0=str->charAt(i);
        i++;
        code1=str->charAt(i);
        c = (h2int(code0) << 4) | h2int(code1);
        encodedString+=c;
      } else{
         encodedString+=c;  
      }
      yield();
    }
    
   return encodedString;
}

 
