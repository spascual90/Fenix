/*
 * ActuatorCalculus.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "ActuatorManager.h"

ActuatorManager::ActuatorManager(double Kp, double Ki, double Kd, int ControllerDirection, int MRA, int error, int deltaCenterOfRudder, int minFeedback, int maxFeedback)
	: PID_ext(&_Input, &_Output, &_Setpoint, Kp, Ki, Kd, ControllerDirection),
	RudderFeedback(MRA, error, deltaCenterOfRudder, minFeedback, maxFeedback)
{

	// Implement PID gain initial values
	SetOutputLimits(getMinRudder(), getMaxRudder());
	//Default Sample Time is 100 millisecs //TODO: Allow user to change sample time
	setSetpoint(0);
	// SPPrima =0 //If always 0 it is not necessary to update value...

	_KpIni = Kp;
	_KiIni = Ki;
	_KdIni = Kd;

}

ActuatorManager::~ActuatorManager() {
	// TODO Auto-generated destructor stub
}


void ActuatorManager::setup(){
	int feedback = updateCurrentRudder();
	bool out_min = feedback < getLimitMinFeedback();
	bool out_max = feedback > getLimitMaxFeedback();

	if (out_min) {
		setDir(EXTEND);
		DEBUG_print("Warning: Linear actuator below limit\n");
	}

	if (out_max) {
		setDir(RETRACT);
		DEBUG_print("Warning: Linear actuator over limit\n");
	}

}


void ActuatorManager::SetMode(int Mode)
{
	//http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
	//Modifies initialization method.
	_Output=0;
	_Input=0;
	PID_ext::SetMode(Mode);
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
		static float prevPIDerrorPrima = PIDerrorPrima;
		setInput (PIDerrorPrima); // should be a value between -/+180. Fn does not check it!!!
		//setSetpoint(0); If always 0 it is not necessary to update value...
		PID_ext::Compute();

		dbt.calculateDBTrim(PIDerrorPrima, getCurrentRudder());

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

	#ifdef DEBUG
	static bool point=false;
	#endif

	if (deadband) target_rudder = trim;

	delta = getSpeed()==0?target_rudder - lastStopRudder:target_rudder - getCurrentRudder();

	if ((abs(delta)<toRudder(getErrorFeedback ())) or
		(out_min and getDir()==RETRACT) or
		(out_max and getDir()==EXTEND)) { //WHATEVER SPEED, IF DELTA IS SMALL OR ACTUATOR ARRIVES TO LIMIT STOP ACTUATOR

			if (getSpeed()!=0) {
				setSpeed (0);
				lastStopRudder=getCurrentRudder();
				#ifdef DEBUG
				sprintf(DEBUG_buffer,"|%i\n",lastStopRudder);
				DEBUG_print(DEBUG_buffer);
				delay(10);
				point=false;
			} else if (!point) {
				point=true;
				DEBUG_print(".");
				delay(10);
				#endif
			}

		return delta;
	}

	e_dir dir = delta==abs(delta)?EXTEND:RETRACT;

	#ifdef DEBUG
	DEBUG_print((dir==EXTEND?">":"<"));
	delay(10);
	point=false;
	#endif

	if (dir!=getDir()) setDir(dir);
	if (getSpeed()!=SPEED_CRUISE) setSpeed (SPEED_CRUISE);

	return delta;
}

void ActuatorManager::ResetTunings(){
	SetTunings (_KpIni, _KiIni, _KdIni);
}
