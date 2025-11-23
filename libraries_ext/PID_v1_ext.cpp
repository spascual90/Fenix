#include "PID_v1_ext.h"
#include "Arduino.h"
#include "GPSport.h"

// COMPUTE_MODEL: PID_BASIC2, PID_ADVANCED
#define  PID_ADVANCED1

PID_ext::PID_ext(double* Input, double* Output, double* Setpoint,
        double Kp, double Ki, double Kd, int ControllerDirection, float speed_ref)
:PID(Input, Output, Setpoint,
        Kp, Ki, Kd, ControllerDirection){
	setSpeed_ref(speed_ref);

}

// overwrite
/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID_ext::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}

// overwrite
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID_ext::Initialize()
{
	_dInput_prev = 0;
    _kdContrib_prev = 0;
	PID::Initialize();
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

#ifdef PID_BASIC
bool PID_ext::Compute(int rudder_error, float speed) {
	if(!inAuto) return false;
	unsigned long now = millis();
	unsigned long timeChange = (now - lastTime);
	if(timeChange>=SampleTime) {
	   /*Compute all the working error variables*/
		  double input = *myInput;
	   double error = *mySetpoint - input;
	   ITerm+= (ki * error);
	   if(ITerm > outMax) ITerm= outMax;
	   else if(ITerm < outMin) ITerm= outMin;
	   double dInput = (input - lastInput);

	   double output = kp * error + ITerm -kd * dInput;
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
#endif

#ifdef PID_ADVANCED1
bool PID_ext::Compute(int rudder_error, float speed)
{
    if (!inAuto) return false;
    static bool timon_en_posicion = false;
    static double error_prev = 1.0;
    static double derr_prev = 0.0;
    static double avg_rudder_error = 0.0;
    static double output_prev = 0.0;
    static double factor = 1.0;
    static bool clamp_I = false;

    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);

    if (timeChange >= SampleTime) {
        /* Entrada y error */
        double input = *myInput;
        double error = *mySetpoint - input;

        /* Deadband: aplica zona muerta proporcional */
        if (calcDeadband(error, error_prev)) {
        	return false;
        }

        /* Escalado dinámico de ganancias */
        factor = (getSpeed_ref() / fmax(speed, 1))* ALFA_10 + ALFA_90 * factor;
        factor = fmin (factor, abs(outMax/(kp*MAX_PID_TURN)));
        double kp_eff = kp * factor;
        double ki_eff = ki * factor;
        double kd_eff = kd / factor;
        double outMaxeff = fmin (outMax * factor, outMax);
        double outMineff = fmax (outMin * factor, outMin);
        //TODO: Si no se modifica se puede simplificar
        double IoutMaxeff = IoutMax;
        double IoutMineff = IoutMin;

        double error_eff = error;

        // Si dentro de la banda salimos de compute al principio
        //if (isInDeadband()) {
        //    error_eff = 0; // dentro de la banda, no actuamos

        //} else {
        //    error_eff = error - _deadband * ((error > 0) ? 1 : -1);
        //}
        error_eff = error - _deadband * ((error > 0) ? 1 : -1);

        _kpContrib = kp_eff * error_eff;

        /* Estado del actuador */
        bool actuator_busy = abs(rudder_error) > I_ACTUATOR_BUSY_THRESHOLD;

        if (!actuator_busy && !timon_en_posicion) {
            timon_en_posicion = true;
        } else if (actuator_busy) {
            timon_en_posicion = false;
        }

        /* Velocidad de cambio de error */
        double derr = (error_eff - error_prev);
        derr = derr * ALFA_10 + ALFA_90 * derr_prev ;

        avg_rudder_error = double (rudder_error) * ALFA_10 + ALFA_90 * avg_rudder_error;

        bool sin_reaccion = ((fabs(derr) < D_DERR_UMBRAL) && (abs (avg_rudder_error) <= D_RUDDER_ERROR_UMBRAL));

        /* Condición de integración */
        bool permitir_integral = !(clamp_I or isInDeadband());

        if (timon_en_posicion && !sin_reaccion) permitir_integral = false;
        //DEBUG_print("DEBUG:--\n");
        //if (permitir_integral) DEBUG_print("DEBUG:permitir_integrar\n");
        if (permitir_integral) ITerm += (ki_eff * error_eff);

        /* Límite integral */
        if (ITerm > IoutMaxeff) ITerm = IoutMaxeff;
        else if (ITerm < IoutMineff) ITerm = IoutMineff;

        /* Derivada filtrada */
        double dInput = (input - lastInput);
        dInput = dInput * ALFA_20 + ALFA_80 * _dInput_prev ;

        _kdContrib = (-kd_eff * dInput) * ALFA_20 + _kdContrib_prev * ALFA_80;
        _kdContrib_prev = _kdContrib;
        /* Salida PID */
        double output_libre = _kpContrib + ITerm + _kdContrib;
        *myOutput = output_libre;
        bool saturated = false;
        if (output_libre > outMaxeff) {
        	*myOutput = outMaxeff;
        	saturated = true;
        } else if (output_libre < outMineff) {
        	*myOutput = outMineff;
        	saturated = true;
        }

        /* Anti-windup */
        // Código siempre despues de calcular Salida PID
        // Atención: al_mismo_lado=true en caso de que error_eff y MyOutput tengan signos opuestos!
        bool al_mismo_lado = !(((error_eff > 0 ? 1 : -1) * (*myOutput > 0 ? 1 : -1)) == 1);
        clamp_I = ((saturated && al_mismo_lado) || actuator_busy);

        //int l=8, d=4;
        //char c3[l+3];
        //char c4[l+3];
        //char c5[l+3];
        //sprintf(DEBUG_buffer,"DEBUG:kd*factor=kd_eff %s*%s=%s\n",dtostrf(kd,l,d,c3),dtostrf(factor,l,d,c4),dtostrf(kd_eff,l,d,c5));
		//sprintf(DEBUG_buffer,"DEBUG:dInput*-kd_eff=_kdContrib %s*%s=%s\n",dtostrf(dInput,l,d,c3),dtostrf(-kd_eff,l,d,c4),dtostrf(_kdContrib,l,d,c5));
        //DEBUG_print();

        // --- Implementación de anti-windup por back-calculation ---

        //if (clamp_I) DEBUG_print("DEBUG:clamp_I\n");
        if (clamp_I && saturated && ITerm!=0) {
        	//Reducimos ITerm lentamente
			ITerm = ITerm * 0.99;
        }

        /* Siguiente ciclo */
        lastInput = input;
        lastTime = now;
        error_prev = error_eff;
        derr_prev = derr;
        _dInput_prev = dInput;

        return true;
    }
    return false;
}

// TRUE: El barco está dentro del deadband dinámico
// FALSE: El barco está fuera del deadband dinámico
bool PID_ext::calcDeadband(double error, double error_prev)
{

	double static error_var = 0;

    switch (_type_DB) {
    case MAXDB:
    	_deadband = MAX_DEADBAND;
    	break;
    case MINDB:
    	_deadband = MIN_DEADBAND;
    	break;
    case AUTODB:
        // estimar nivel de ruido con diferencia de error
    	error_var = (ALFA_001 * fabs(error - error_prev) + ALFA_999 * error_var);
    	_deadband =  error_var * FACTOR_DEADBAND;
    	_deadband = fmax(MIN_DEADBAND, _deadband);
    	_deadband = fmin(MAX_DEADBAND, _deadband);
    	break;
    }
    _inDeadband = (fabs(error) <= _deadband);
    return _inDeadband;
}

#endif





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

