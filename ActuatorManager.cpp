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
	updateCurrentRudder();
	setTargetRudder(getCurrentRudder());
	bool out_min = getCurrentFeedback() < getLimitMinFeedback();
	bool out_max = getCurrentFeedback() > getLimitMaxFeedback();

	if (out_min) {
		setDir(EXTEND);
		DEBUG_print(F("W: L.actuator below limit\n"));
	}

	if (out_max) {
		setDir(RETRACT);
		DEBUG_print(F("W: L.actuator over limit\n"));
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
	int target = getTargetRudder();
	int current = getCurrentRudder();
	if (abs(delta_rudder) > abs (target-current))
	{
		target += delta_rudder;
		setTargetRudder(target);
	}
	controlActuator (getTargetRudder());

	return 1;
}

int ActuatorManager::controlActuator(int target_rudder_deg)
{
    // --- Estado actual ---
    //int feedback = updateCurrentRudder();
    //int current = getCurrentRudder();
    int delta = target_rudder_deg - getCurrentRudder();

	#ifdef DEBUG
    DEBUG_sprintf("tr,c,d",target_rudder_deg, getCurrentRudder(), delta);
	#endif

    static unsigned long t_last_dir_change = 0;
    static uint8_t estados =0;
    static uint8_t estado_timer =0;
	static bool double_check = false;

    const int deadband = toRudder(getErrorFeedback());
    const unsigned long change_delay = ACTUATOR_STOP_TIME; // 300–500 ms típico
    const uint8_t slow_threshold = 30;   // grados para entrar en modo SLOW
    const uint8_t stop_threshold = toRudder(getErrorFeedback ());   // en este rango paramos
    const uint8_t restart_threshold = stop_threshold * 10;

    bool min_limit = getCurrentFeedback() < getLimitMinFeedback();
    bool max_limit = getCurrentFeedback() > getLimitMaxFeedback();
    int new_speed;
    int speed_cmd;
	int current_speed = getSpeed();



    // --- 1. Dirección deseada ---
    e_dir desired_dir = (delta >= 0 ? EXTEND : RETRACT);
    static e_dir last_direction = getDir(); //1-desired_dir; // sólo pasa por aquí la primera vez

	//--- timer
    unsigned long now = millis();
	if (current_speed ==0)
	{
		if (estado_timer==0){
			estado_timer=1; //inicia timer
			t_last_dir_change = now;//reset timer
		}
	}

	if (estado_timer==1)
	{
		if (now - t_last_dir_change < change_delay)
		{
			#ifdef DEBUG
			DEBUG_print("t.w\n");//waiting
			delay(10);
			#endif
		}
		else
		{
			#ifdef DEBUG
			DEBUG_print("t.f\n");// finished
			delay(10);
			#endif
			estado_timer = 2;
		}
	}

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
    if (abs(delta) <= (stop_threshold))
    {

		#ifdef DEBUG
		DEBUG_sprintf("3.stop\n");
		delay(10);
		#endif
		setSpeed(0, true);
        return delta;
    }

	if (current_speed ==0 and restart_threshold > abs(delta))
	{
		#ifdef DEBUG
		DEBUG_sprintf("3.dont_start\n");
		delay(10);
		#endif
		setSpeed(0, true);
		return delta;
	}

    // --- 4. Cambio de dirección protegido ---
    if ((desired_dir != last_direction) or (estados!=0))
    {
    	speed_cmd=0;
    	switch (estados) {
		case 0: // en "case 0" sólo entra por cambio de dirección
			//protector de error en lectura POT
			if (double_check==false)
			{
				double_check=true;
				#ifdef DEBUG
				DEBUG_sprintf("****** 4.!b1", delta);
				#endif
				return delta;
			}
			double_check=false;
			#ifdef DEBUG
			DEBUG_sprintf("****** 4.!b2", delta); //begin
			delay(10);
			#endif
			estados=1;
			break;
		case 1:
    		if (current_speed==0)
    		{
    			estados=2;

				#ifdef DEBUG
				DEBUG_print("4.!w\n");
				delay(10);
				#endif

				//t_last_dir_change = now;
			}
    		break;
		case 2:
			if (estado_timer==2)//(now - t_last_dir_change < change_delay)
			{
				#ifdef DEBUG
				DEBUG_print("4.f\n");// finished
				delay(10);
				#endif
				estados = 0;
				estado_timer=0;
			}
		break;
    	}
    }

    if (estados==0)
    {
		// --- 5. Elegir velocidad según magnitud del error ---


		if (abs(delta) > slow_threshold)
		{
			speed_cmd = SPEED_CRUISE;   // error grande - mover rápido
			#ifdef DEBUG
			DEBUG_sprintf("5.SC", delta);
			delay(10);
			#endif

		} else
		{
			// si actualmente parado y el error es pequeño, seguir parado
			if (current_speed ==0 and restart_threshold > abs(delta))
			{
				speed_cmd = 0;     // error pequeño y parado- no mover aún
				#ifdef DEBUG
				DEBUG_sprintf("5.0", delta);
				delay(10);
				#endif
				return delta;
			} else {
				speed_cmd = SPEED_CRUISE/3;     // error pequeño y moviendose - aproximación suave
				#ifdef DEBUG
				DEBUG_sprintf("5./3", delta);
				delay(10);
				#endif
			}
		}
    } // if (estados==0)

	// --- 6. Aplicar dirección y velocidad con rampas ---
	if ((last_direction != desired_dir) and (estados==0))

	{
		#ifdef DEBUG
		DEBUG_sprintf("6.c.dd,gd",desired_dir,getDir());
		delay(10);
		#endif
		setDir(desired_dir);
		last_direction= desired_dir;
	}
    // Rampa de velocidad real: suavizar pasos
	int delta_speed = speed_cmd - current_speed;

	if (abs(delta_speed) < 5)
	{
		delta_speed = 0;
		//DEBUG_sprintf("cmd,crt",speed_cmd, current_speed);
	}

    if (delta_speed > 0) //(speed_cmd > current_speed)
    {
        new_speed = current_speed + 5;  // rampa de aceleración
		#ifdef DEBUG
		DEBUG_print("6.a\n");
		delay(10);
		#endif
    } else
    {
    	if (delta_speed < 0 )
    	{
			new_speed = current_speed - 5;  // rampa de frenado
			#ifdef DEBUG
			DEBUG_print("6.f\n");
			delay(10);
			#endif
		} else  //delta_speed == 0
		{
			new_speed = current_speed;
			#ifdef DEBUG
			DEBUG_print("6.0\n");
			delay(10);
			#endif
			//return delta;
		}
    }
    // Limitar velocidad final
    if (new_speed < 5) new_speed = 0;
    if (SPEED_CRUISE < new_speed ) new_speed = SPEED_CRUISE;

    if (new_speed!=current_speed)
    {
		#ifdef DEBUG
    	DEBUG_sprintf("speed,estados", new_speed, estados);
		#endif
    	setSpeed(new_speed, true);
    }

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
