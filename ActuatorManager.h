/*
 * ActuatorCalculus.h
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#ifndef ACTUATORMANAGER_H_
#define ACTUATORMANAGER_H_

#include "PID_v1_ext.h"
#include "RudderFeedback.h"
#include "CurrentFeedback.h"
#include "ActuatorController.h"
#include "FossenReferenceModel.h"
#include "ReferenceFeedforward.h"

// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

// All configurations are managed in Fenix_config.h

// INSTALATION PARAMETERS: POSSIBLE VALUES

// Rudder gain
#define RUD_B_MIN 0.2 	//Fast boats
#define RUD_B_MED 1 	//Cruisers
#define RUD_B_MAX 1.5 	//Sailing

#define ACTUATOR_STOP_TIME 1800 //stop time between changes of direction

// TODO: Installation side always STARBOARD. Implement PORTBOARD
// Installation side

typedef enum type_instSide {STARBOARD, PORTBOARD} type_instSide;

class ActuatorManager: public PID_ext,
		public RudderFeedback,
		public ActuatorController,
		public CurrentFeedback
{
public:
	ActuatorManager(double Kp, double Ki, double Kd, int ControllerDirection, int MRA, int error, int deltaCenterOfRudder, int minFeedback, int maxFeedback, float refSpeed, int maxCurrent);
	virtual ~ActuatorManager();
	void setup(void);
	void startAutoMode(float currentHeadingT);
	void stopAutoMode(float currentHeadingT);
	void changeDesiredHeadingT(float targetHeadingT);
	//int Compute(float setPoint, float processVariable, float speed, float predictedYaw);
	//int Compute(float PIDerrorPrima, float speed);
	int Compute(float TargetBearing, float CurrentHeadingT, float boatSpeed);
	int compute_VA(void);
	int controlActuator (int target);
	void ResetTunings();

	double getOutput() const {
		return _Output;
	}

	void setInput(double input) {
		_Input = input;
	}

	double getInput() const {
		return _Input;
	}


	void setSetpoint(double setpoint) {
		_Setpoint = setpoint;
	}

	inline int getTargetRudder() const {
		return _targetRudder;
	}

	void setTargetRudder(int targetRudder) {

		if (targetRudder > getMaxRudder())  {targetRudder = getMaxRudder();}
		else if (targetRudder < getMinRudder()) {targetRudder = getMinRudder();}
		//DEBUG_sprintf("_targetRudder",_targetRudder);
		_targetRudder = targetRudder;
	}

	type_instSide getInstallationSide() const {
		return _installation_side;
	}

	void setInstallationSide(type_instSide installationSide = STARBOARD) {
		_installation_side = installationSide;
	}

	void setDirectDeltaParams(float Kffeff, float Teff);
//	float getKeff() const {return nomotoId.getKeff();	}
//	float getTeff() const {return nomotoId.getTeff();	}
//	float getKffeff() const {return nomotoId.getKffeff();	}


protected:
	int changeRudder(int delta_rudder);
	bool checkBlockage(void);

private:
	//PID: Define Variables we'll be connecting to
	double _Setpoint, _Input, _Output;

	double _KpIni, _KiIni, _KdIni;

	// target rudder in manual mode
	int _targetRudder=1;

	//Installation side
	type_instSide _installation_side = STARBOARD;

	void SetMode(int Mode);
	FossenReferenceModel referenceModel;
	ReferenceFeedforward feedForward;

	float reduce360(float value) {
		value = fmod (value, 360.0f);
		if (value<0) value+= 360.0f;
		return value;
	}

	float reduce180(float value) {
	    // Normalizar primero al rango [0, 360)
		value = reduce360(value);
	    // Convertir a rango [-180, 180)
	    if (value >= 180.0f) value -= 360.0f;
	    return value;
	}

	float delta180(float angle1, float angle2) {
	    if (angle1 < 0.0f || angle1 >= 360.0f) return -360.0f;
	    if (angle2 < 0.0f || angle2 >= 360.0f) return -360.0f;

	    return reduce180(angle1 - angle2);
	}

};

#endif /* ACTUATORMANAGER_H_ */
