/*
 * ActuatorCalculus.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "ActuatorManager.h"

ActuatorManager::ActuatorManager(double Kp, double Ki, double Kd, int ControllerDirection, int MRA, int error, int deltaCenterOfRudder, int minFeedback, int maxFeedback, float refSpeed)
	: PID_ext(&_Input, &_Output, &_Setpoint, Kp, Ki, Kd, ControllerDirection, refSpeed),
	  PID_ATune(&_ATune_Input, &_ATune_Output),
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


void ActuatorManager::setup(void){
	int feedback = updateCurrentRudder();
	bool out_min = feedback < getLimitMinFeedback();
	bool out_max = feedback > getLimitMaxFeedback();

	if (out_min) {
		setDir(EXTEND);
		DEBUG_print(F("W: L.actuator below limit\n"));
	}

	if (out_max) {
		setDir(RETRACT);
		DEBUG_print(F("W: L.actuator over limit\n"));
	}

	//setupAutoTune(aTuneNoise, aTuneStep, aTuneLookBack);

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

}

void ActuatorManager::stopAutoMode(){
	//turn the PID on
	SetMode(MANUAL);
	setTargetRudder(getCurrentRudder());
}

int ActuatorManager::Compute(float setPoint, float processVariable, float speed, float predictedYaw) {

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
//			In deadband: To be defined

		float PIDerrorPrima = setPoint- processVariable;
		if (PIDerrorPrima>180) PIDerrorPrima -=360;
		if (PIDerrorPrima<=-180)  PIDerrorPrima +=360;

		return this->Compute(PIDerrorPrima, speed, predictedYaw);
}

int ActuatorManager::Compute(float PIDerrorPrima, float speed, float predictedYaw) {

		static int delta_rudder =0;
		setInput (PIDerrorPrima); // should be a value between -/+180. Fn does not check it!!!
		//setSetpoint(0); If always 0 it is not necessary to update value...

		PID_ext::Compute(delta_rudder, speed);
		delta_rudder = controlActuator (getOutput());
		PID_ext::setKanticipContrib(predictedYaw/100.0);
		//predictedYaw

		return 1;
	}

int ActuatorManager::Compute_Autotune(float PIDerrorPrima) {

		//setATuneInput (PIDerrorPrima); // should be a value between -/+180. Fn does not check it!!!
		//setSetpoint(0); If always 0 it is not necessary to update value...

		//evaluateAutoTune();

		//controlActuator (int(_ATune_Output), false, 0);

		return 1;
	}

int ActuatorManager::compute_VA() {
	// Refresh virtual actuator status
	#ifdef VIRTUAL_ACTUATOR
	int distance = ActuatorController::compute_VA();
	int new_analog = getVAanalogRead() + distance;
	if (new_analog>getLimitMaxFeedback()) new_analog = getLimitMaxFeedback();
	if (new_analog<getLimitMinFeedback()) new_analog = getLimitMinFeedback();

	setVAanalogRead(new_analog);
	#endif
}

int ActuatorManager::changeRudder(int delta_rudder) {

	int target = getTargetRudder() + delta_rudder;
	setTargetRudder(target);

	controlActuator (getTargetRudder());

	return 1;
}

//Returns delta btw current and target rudder position
//int ActuatorManager::controlActuator(int target_rudder) {
//	int delta = 0;
//	int feedback = updateCurrentRudder();
//	static int lastStopRudder = getCurrentRudder();
//	static bool wait_after_change = false;
//	static e_dir lastDir= EXTEND;
//	static unsigned long change_time = 0;
//	bool out_min = feedback < getLimitMinFeedback();
//	bool out_max = feedback > getLimitMaxFeedback();
//	unsigned long ahora;
//	//if (out_min) DEBUG_print(F("Out of LimitMinFeedback. Recalibrate linear actuator\n"));
//	//if (out_max) DEBUG_print(F("Out of LimitMaxFeedback. Recalibrate linear actuator\n"));
//	#ifdef DEBUG
//	static bool point=false;
//	#endif
//
//	delta = getSpeed()==0?target_rudder - lastStopRudder:target_rudder - getCurrentRudder();
//	e_dir dir = delta==abs(delta)?EXTEND:RETRACT;
//	ahora = millis();
//
//	if (((ahora-change_time) > ACTUATOR_STOP_TIME) and (wait_after_change==true)) {
//		wait_after_change=false;
//		#ifdef DEBUG
//		DEBUG_print(F("G"));
//		#endif
//	}
//	bool limite = (out_min and dir==RETRACT) or (out_max and dir==EXTEND);
//	if ((abs(delta)<toRudder(getErrorFeedback ())) or
//		limite or
//		(wait_after_change==true)) { //WHATEVER SPEED, IF DELTA IS SMALL OR ACTUATOR ARRIVES TO LIMIT STOP ACTUATOR
//		//DEBUG_print(F("_"));
//			if (getSpeed()!=0) {
//				setSpeed (0, limite);
//				lastStopRudder=getCurrentRudder();
//				#ifdef DEBUG
//				sprintf(DEBUG_buffer,"|%i\n",lastStopRudder);
//				DEBUG_print(DEBUG_buffer);
//				delay(10);
//				point=false;
//			} else if (!point) {
//				point=true;
//				DEBUG_print(F("."));
//				delay(10);
//				#endif
//			}
//
//		return delta;
//	}
//
//	if (dir!=lastDir and wait_after_change==false) {
//		wait_after_change=true;
//		change_time = ahora;
//		#ifdef DEBUG
//		DEBUG_print(F("S"));
//		#endif
//		lastDir = dir;
//		return delta;
//	}
//
//	#ifdef DEBUG
//	DEBUG_print((dir==EXTEND?">":"<"));
//	delay(10);
//	point=false;
//	#endif
//
//	if (dir!=getDir()) setDir(dir);
//	if (getSpeed()!=SPEED_CRUISE) setSpeed (SPEED_CRUISE);
//
//	return delta;
//}

int ActuatorManager::controlActuator(int target_rudder_deg)
{
    // --- Estado actual ---
    int feedback = updateCurrentRudder();
    int current = getCurrentRudder();
    int delta = target_rudder_deg - current;

    static unsigned long t_last_dir_change = 0;
    static e_dir last_direction = EXTEND;
    static bool waiting_after_change = false;

    const int deadband = toRudder(getErrorFeedback());
    const unsigned long change_delay = ACTUATOR_STOP_TIME; // 300–500 ms típico
    const int slow_threshold = 10;   // grados para entrar en modo SLOW
    const int stop_threshold = toRudder(getErrorFeedback ());   // en este rango paramos

    unsigned long now = millis();

    bool min_limit = feedback < getLimitMinFeedback();
    bool max_limit = feedback > getLimitMaxFeedback();


    // --- 1. Dirección deseada ---
    e_dir desired_dir = (delta >= 0 ? EXTEND : RETRACT);

    // --- 2. Protección por límites físicos ---
    if ((desired_dir == RETRACT && min_limit) ||
        (desired_dir == EXTEND  && max_limit))
    {
    	#ifdef DEBUG
    	DEBUG_print("2.\n");
    	delay(10);
    	#endif

        setSpeed(0, true);
        return delta;
    }

    // --- 3. Deadband: parar para evitar hunting ---
    if (abs(delta) <= stop_threshold)
    {
		//#ifdef DEBUG
		//DEBUG_print("3.\n");
		//delay(10);
		//#endif

        setSpeed(0, true);
        return delta;
    }

    // --- 4. Cambio de dirección protegido ---
    if (desired_dir != last_direction)
    {

        if (!waiting_after_change)
        {
			#ifdef DEBUG
			DEBUG_print("4.!w\n");
			delay(10);
			#endif

            waiting_after_change = true;
            t_last_dir_change = now;
            setSpeed(0, true);
            return delta;
        }
    }

    if (waiting_after_change)
    {
        if (now - t_last_dir_change < change_delay)
        {
			#ifdef DEBUG
			DEBUG_print("4.w\n");
			delay(10);
			#endif
            setSpeed(0, true);
            return delta;
        }
        else
        {
            waiting_after_change = false;
            last_direction = desired_dir;
        }
    }

    // --- 5. Elegir velocidad según magnitud del error ---
    int speed_cmd;

    if (abs(delta) > slow_threshold)
        speed_cmd = SPEED_CRUISE;   // error grande - mover rápido
    else
        speed_cmd = SPEED_CRUISE/2;     // error pequeño - aproximación suave

    // --- 6. Aplicar dirección y velocidad con rampas ---
    if (getDir() != desired_dir)
        setDir(desired_dir);

    // Rampa de velocidad real: suavizar pasos
    int current_speed = getSpeed();
    int new_speed = current_speed;

    if (speed_cmd > current_speed)
        new_speed = current_speed + 10;  // rampa de aceleración
    else if (speed_cmd < current_speed)
        new_speed = current_speed - 10;  // rampa de frenado

    // Limitar velocidad final
    new_speed = constrain(new_speed, 0, SPEED_CRUISE);

	#ifdef DEBUG
	DEBUG_print("6.\n");
	delay(10);
	#endif

    setSpeed(new_speed, true);

    return delta;
}

void ActuatorManager::ResetTunings(){
	SetTunings (_KpIni, _KiIni, _KdIni);
}


// AUTOTUNE
//input: bearing
//input noise band: max. bandwith: 5 degrees
//look back time (local peaks filtering): 5 seg
//
//The number of cycles performed will vary between 3 and 10.
//The algorithm waits until the last 3 maxima have been within 5% of each other.
//This is trying to ensure that we’ve reached a stable oscillation and there’s no external strangeness happening. This leads me to…
//
//output: target rudder angle
//
//proceso:
//- En modo Auto, navegar hacia un rumbo estable
//- Extender actuador al máximo hasta alcanzar un rumbo 100º a estribor
//- Retraer actuador al mínimo hasta alcanzar un rumbo 200º a babor
//- Extender actuador al máximo hasta alcanzar un rumbo 200º a estribor
//- Repetir hasta que Autotune considere necesario (3/10 veces)
//
//Monitorización:
//Target bearing en cada momento
//Numero de ciclos
//Estado del proceso: Starting process, turn starboard, turn portboard, process finished successfully, process finished with errors

void ActuatorManager::setupAutoTune(double aTuneNoise, double aTuneStep, double aTuneLookBack)
{
    SetNoiseBand(aTuneNoise);
    SetOutputStep(aTuneStep);
    SetLookbackSec((int)aTuneLookBack);
}


void ActuatorManager::startAutoTune(void) {
 if(!_tuning)
  {
    //Set the output to the desired starting frequency.

    _tuning = true;
	DEBUG_print(F("!ATune started\n"));
  }
}

bool ActuatorManager::evaluateAutoTune(void) {
	int l=8, d=4;
	char c3[l+3];
	char c4[l+3];
	char c5[l+3];

	if(_tuning) {
		  byte val = (Runtime());
		  if (val!=0) {
			DEBUG_print(F("!Autotune finished\n"));
			stopAutoTune();

			sprintf(DEBUG_buffer,"Kp, Ki, Kd: %s, %s, %s\n",dtostrf(PID_ATune::GetKp(),l,d,c3) ,dtostrf(PID_ATune::GetKi(),l,d,c4),dtostrf(PID_ATune::GetKd(),l,d,c5));
			DEBUG_print();

		  }
		  return _tuning;
	}
}


void ActuatorManager::stopAutoTune(void) {
	if (_tuning) {
		//cancel autotune
		Cancel();
		_tuning = false;
		_ATune_Output = 0;
		_ATune_Input = 0;
		DEBUG_print(F("!ATune stopped\n"));

	}
}

bool ActuatorManager::CopyToPIDAutoTune(void) {

	if(!_tuning) {
		//we're done, set the tuning parameters
		double l_kp = PID_ATune::GetKp();
		double l_ki = PID_ATune::GetKi();
		double l_kd = PID_ATune::GetKd();
		SetTunings(l_kp,l_ki,l_kd);
	}
	return (!_tuning);
}
