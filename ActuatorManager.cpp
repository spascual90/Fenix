/*
 * ActuatorCalculus.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "ActuatorManager.h"

ActuatorManager::ActuatorManager(double Kp, double Ki, double Kd, int ControllerDirection, int MRA, int error, int deltaCenterOfRudder, int minFeedback, int maxFeedback)
	: PID(&_Input, &_Output, &_Setpoint, Kp, Ki, Kd, ControllerDirection),
	RudderFeedback(MRA, error, deltaCenterOfRudder, minFeedback, maxFeedback)
{

	// Implement PID gain initial values
	SetOutputLimits(getMinRudder(), getMaxRudder());
	SetSampleTime(1000); //in millisecs //TODO: Allow user to change sample time
	setSetpoint(0); // SPPrima =0 //TODO: If always 0 it is not necessary to update value...

	_KpIni = Kp;
	_KiIni = Ki;
	_KdIni = Kd;

}

ActuatorManager::~ActuatorManager() {
	// TODO Auto-generated destructor stub
}

void ActuatorManager::startAutoMode(){
	//turn the PID on
	SetMode(AUTOMATIC);
	dbt.StartSampling();

}

void ActuatorManager::stopAutoMode(){
	//turn the PID on
	SetMode(MANUAL);
	setTargetRudder(getCurrentRudder());
}


int ActuatorManager::Compute(float setPoint, float processVariable) {

//			PID FACTOR
//		  1. Target Bearing(TB) is considered the 0 reference (TB' reference system) instead of magnetic North.
//			To express PV and SP in TB reference system,
//		 		Process Variable (PV') = PV-SP (also expressed as CB-TB (Current Bearing with reference Target Bearing))
//		 		Setpoint (SP') = SP-SP=0
//
//		  2. In TB reference system, Process Variable (PV') is expressed as an angle from -180 to 180
//			if PV'<=180 then PV'=PV'+360
//			if PV'>180 then PV'=PV'-360
//
//			3. In TB reference system,
//		 		PIDError' = SP'-PV'=0-PV'=-PV'=SP-PV.
//			Out of deadband: Target Rudder = PIDError'
//			In deadband: Target Rudder = Autotrim contribution

		float PIDerrorPrima = setPoint- processVariable;
		if (PIDerrorPrima>180) PIDerrorPrima -=360;
		if (PIDerrorPrima<=-180)  PIDerrorPrima +=360;

		return this->Compute(PIDerrorPrima);
}

int ActuatorManager::Compute(float PIDerrorPrima) {
		setInput (PIDerrorPrima); // should be a value between -/+180. Fn does not check it!!!
		//setSetpoint(0); If always 0 it is not necessary to update value...
		PID::Compute();

		dbt.calculateDBTrim(PIDerrorPrima, getCurrentRudder());

/*#ifdef DEBUG_PORT
		// deadband and trimming
		static bool deadband, prev_deadband;
		int l=8, d=2;
		char c4[l+3];

		deadband=dbt.getDeadband(PIDerrorPrima);
		if (prev_deadband!=deadband) {
			if (deadband) {
				sprintf(DEBUG_buffer,"In deadband (HDG-CTS<%i). Trim: %s (Rudder:%i )\n", dbt.getDeadband(), dtostrf(dbt.getTrim(),l,d,c4), getCurrentRudder());
				DEBUG_print();
			}
			if (!deadband) {
				sprintf(DEBUG_buffer,"Out deadband (HDG-CTS>%i). Rudder:%i\n", dbt.getDeadband(), getCurrentRudder());
				DEBUG_print(DEBUG_buffer);

			}
			DEBUG_PORT.flush();
		}
		prev_deadband=deadband;
#endif*/

		controlActuator (getOutput(), dbt.getDeadband(PIDerrorPrima), int(dbt.getTrim()));

		return 1;
	}

int ActuatorManager::changeRudder(int delta_rudder) {

	int target = getTargetRudder() + delta_rudder;
	setTargetRudder(target);

	controlActuator (getTargetRudder());

	return 1;
}

int ActuatorManager::controlActuator(int target_rudder, boolean deadband, int trim) {
	int delta = 0;
	int feedback = updateCurrentRudder();
	static int lastStopRudder = getCurrentRudder();
	bool out_min = feedback < getLimitMinFeedback();
	bool out_max = feedback > getLimitMaxFeedback();
	//static bool point=false;

	if (deadband) target_rudder = trim;

	delta = getSpeed()==0?target_rudder - lastStopRudder:target_rudder - getCurrentRudder();
	if ((abs(delta)<toRudder(getErrorFeedback ())) or
		(out_min and getDir()==RETRACT) or
		(out_max and getDir()==EXTEND)) { //WHATEVER SPEED, IF DELTA IS SMALL OR ACTUATOR ARRIVES TO LIMIT STOP ACTUATOR

			if (getSpeed()!=0) {
				setSpeed (0);
				lastStopRudder=getCurrentRudder();
				//sprintf(DEBUG_buffer,"|%i\n",lastStopRudder);
				//DEBUG_print(DEBUG_buffer);
				//delay(10);
				//point=false;
			}// else if (!point) {
			//	point=true;
			//	DEBUG_print(".");
			//	delay(10);
			//}

		return delta;
	}

	e_dir dir = delta==abs(delta)?EXTEND:RETRACT;

	//DEBUG_print((dir==EXTEND?">":"<"));
	//delay(10);
	//point=false;

	if (dir!=getDir()) setDir(dir);
	if (getSpeed()!=SPEED_CRUISE) setSpeed (SPEED_CRUISE);

	return delta;
}

void ActuatorManager::ResetTunings(){
	SetTunings (_KpIni, _KiIni, _KdIni);
}

void ActuatorManager::SetMode(int Mode)
{
	//http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
	//Modifies initialization method.
	_Output=0;
	_Input=0;
	PID::SetMode(Mode);
}
