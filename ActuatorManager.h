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
#include "ActuatorController.h"
//#include "DeadbandTrim.h"
#include "PID_AutoTune_v0.h"


// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

// All configurations are managed in Fenix_config.h

// INSTALATION PARAMETERS: POSSIBLE VALUES

// Rudder gain
#define RUD_B_MIN 0.2 	//Fast boats
#define RUD_B_MED 1 	//Cruisers
#define RUD_B_MAX 1.5 	//Sailing

#define ACTUATOR_STOP_TIME 500 //stop time between changes of direction

// TODO: Installation side always STARBOARD. Implement PORTBOARD
// Installation side
typedef enum type_instSide {STARBOARD, PORTBOARD} type_instSide;

class ActuatorManager: public PID_ext, public PID_ATune,
		public RudderFeedback,
		public ActuatorController {
public:
	ActuatorManager(double Kp, double Ki, double Kd, int ControllerDirection, int MRA, int error, int deltaCenterOfRudder, int minFeedback, int maxFeedback, float refSpeed);
	virtual ~ActuatorManager();
	void setup(void);
	void startAutoMode();
	void stopAutoMode();
	int Compute(float setPoint, float processVariable, float speed, float predictedYaw);
	int Compute(float PIDerrorPrima, float speed, float predictedYaw);
	int Compute_Autotune(float PIDerrorPrima);
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

	void setupAutoTune(double aTuneNoise, double aTuneStep, double aTuneLookBack);
	void startAutoTune(void);
	void stopAutoTune (void);
	bool evaluateAutoTune (void);
	bool CopyToPIDAutoTune(void);

	void setATuneInput(double aTuneInput) {
		_ATune_Input = aTuneInput;
	}

	//DeadbandTrim
	//DeadbandTrim dbt;

protected:
	int changeRudder(int delta_rudder);

private:
	//PID: Define Variables we'll be connecting to
	double _Setpoint, _Input, _Output;

	double _KpIni, _KiIni, _KdIni;

	// target rudder in manual mode
	int _targetRudder=1;

	//Installation side
	type_instSide _installation_side = STARBOARD;

	//ATunePID
	boolean _tuning = false;
	//ATune PID: Define Variables we'll be connecting to
	double _ATune_Input, _ATune_Output;

	void SetMode(int Mode);

};

#endif /* ACTUATORMANAGER_H_ */
