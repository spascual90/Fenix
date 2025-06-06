#include "PID_v1_ext.h"
#include "Arduino.h"
#include "GPSport.h"
PID_ext::PID_ext(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection)
:PID(Input, Output, Setpoint,
        Kp, Ki, Kd, ControllerDirection){

}


// overwrite
/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_ext::SetOutputLimits(double Min, double Max)
{
   PID::SetOutputLimits(Min, Max);
   IoutMin = outMin/I_MAX_COEFICIENT;
   IoutMax = outMax/I_MAX_COEFICIENT;
}

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
      // SPM INI
      // Prevents windup effect
      //https://youtu.be/NVLXCwc8HzM?si=5R9HTPmz-H-DV_IK
      if (!clamp_I) ITerm+= (ki * error);
      // SPM FIN
      //Limit I contribution
      if(ITerm > IoutMax) ITerm= IoutMax;
      else if(ITerm < IoutMin) ITerm= IoutMin;
      double dInput = (input - lastInput)/(timeChange/SampleTime);
      /*Compute PID Output*/
      // SPM INI
      _kpContrib = kp * error;

      // Low Pass Filter Implementation on derivative
      static double _kdContrib_prev=0;
      _kdContrib = (-kd * dInput)* (1.0-D_FILTER_ALFA) + _kdContrib_prev * D_FILTER_ALFA;

      double output = _kpContrib + ITerm + _kdContrib;
      double output_pre = output;
      // SPM FIN

	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  *myOutput = output;

      /*Remember some variables for next time*/
      lastInput = input;
      lastTime = now;

      // SPM INI
      //anti-windup mechanism
      //https://youtu.be/NVLXCwc8HzM?si=5R9HTPmz-H-DV_IK
      // Saturation check
      bool saturated = (output_pre!=output);
      // TRUE if PID and error have the same sign
      bool equal_sign = (((error>0?1:-1) * (output>0?1:-1))==1);
      clamp_I = (saturated && equal_sign);
      // SPM FIN

	  return true;
   }
   else return false;
}

bool PID_ext::resetITerm(float delta) {
	if (delta>I_MAX_DELTA) {
		resetITerm();
		return true;
	}
	return false;
}

void PID_ext::resetITerm(void) {
	ITerm = 0;
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

	double PID_ext::getSetpoint() {
		return *mySetpoint;
	}

