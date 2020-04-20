#include "PID_v1_ext.h"
#include "Arduino.h"

PID_ext::PID_ext(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
:PID(Input, Output, Setpoint,
        Kp, Ki, Kd, ControllerDirection){

}


// overwrite
void PID_ext::SetTunings(double Kp, double Ki, double Kd) {
	Kp = round(Kp*100)/100.0;
	Ki = round(Ki*100)/100.0;
	Kd = round(Kd*100)/100.0;
	PID::SetTunings(Kp, Ki, Kd);
}


bool PID_ext::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
	  double input = *myInput;
      double error = *mySetpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (input - lastInput);
 
      /*Compute PID Output*/
      // SPM INI
      _kpContrib = kp * error;
      _kdContrib = -kd * dInput;

      double output = _kpContrib + ITerm + _kdContrib;
      // SPM FIN
      
	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  *myOutput = output;
	  
      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;

	  return true;
   }
   else return false;
}

//Display functions ****************************************************************
	double PID_ext::getKdContrib() {
		return _kdContrib;
	}

	double PID_ext::getKpContrib() {
		return _kpContrib;
	}

    unsigned long PID_ext::GetSampleTime() {
    	return PID::SampleTime;
    }
