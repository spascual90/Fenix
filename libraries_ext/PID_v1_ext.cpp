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


//bool PID_ext::Compute()
//{
//   if(!inAuto) return false;
//   unsigned long now = millis();
//   unsigned long timeChange = (now - lastTime);
//   if(timeChange>=SampleTime)
//   {
//      /*Compute all the working error variables*/
//	  double input = *myInput;
//      double error = *mySetpoint - input;
//      // SPM INI
//      // Prevents windup effect
//      //https://youtu.be/NVLXCwc8HzM?si=5R9HTPmz-H-DV_IK
//      if (!clamp_I) ITerm+= (ki * error);
//      // SPM FIN
//      //Limit I contribution
//      if(ITerm > IoutMax) ITerm= IoutMax;
//      else if(ITerm < IoutMin) ITerm= IoutMin;
//      double dInput = (input - lastInput)/(timeChange/SampleTime);
//      /*Compute PID Output*/
//      // SPM INI
//      _kpContrib = kp * error;
//
//      // Low Pass Filter Implementation on derivative
//      static double _kdContrib_prev=0;
//      _kdContrib = (-kd * dInput)* (1.0-D_FILTER_ALFA) + _kdContrib_prev * D_FILTER_ALFA;
//
//      double output = _kpContrib + ITerm + _kdContrib;
//      double output_pre = output;
//      // SPM FIN
//
//	  if(output > outMax) output = outMax;
//      else if(output < outMin) output = outMin;
//	  *myOutput = output;
//
//      /*Remember some variables for next time*/
//      lastInput = input;
//      lastTime = now;
//
//      // SPM INI
//      //anti-windup mechanism
//      //https://youtu.be/NVLXCwc8HzM?si=5R9HTPmz-H-DV_IK
//      // Saturation check
//      bool saturated = (output_pre!=output);
//      // TRUE if PID and error have the same sign
//      bool equal_sign = (((error>0?1:-1) * (output>0?1:-1))==1);
//      clamp_I = (saturated && equal_sign);
//      // SPM FIN
//
//	  return true;
//   }
//   else return false;
//}

//bool PID_ext::Compute(int rudder_error)
//{
//    if (!inAuto) return false;
//
//    unsigned long now = millis();
//    unsigned long timeChange = (now - lastTime);
//
//    if (timeChange >= SampleTime)
//    {
//        /* Lectura de entrada y cálculo de error */
//        double input = *myInput; // rumbo actual
//        double error = *mySetpoint - input;
//
//        _kpContrib = kp * error;
//
//
//        /* --- ANTI-WINDUP EXTENDIDO --- */
//        // Factor para considerar "P cerca del límite" (ej: 0.8 = 80% de la capacidad)
//        const double P_THRESHOLD_FACTOR = 0.80;
//
//        // Magnitud máxima disponible de salida (usa valor absoluto por si outMin < 0)
//        double outMaxAbs = fmax(fabs(outMax), fabs(outMin));
//
//        // Umbral de P en unidades de salida (si |P| >= P_THRESHOLD ésta condición se cumple)
//        double P_threshold = P_THRESHOLD_FACTOR * outMaxAbs;
//
//        // Si el proporcional está en el rango en el que por sí solo ya "está trabajando fuerte"
//        bool P_cerca_del_limite = (fabs(_kpContrib) >= P_threshold);
//
//        // Diferencia entre consigna de timón (output actual del PID) y posición real
//        // Necesitas tener disponible 'rudder_position' (sensor) y el setpoint actual del timón
//        bool actuator_busy = abs(rudder_error) > 10 ;//rudder_threshold; // p.ej. 2.0 grados
//
//        // Prevención de windup
//        if (!clamp_I && !actuator_busy && P_cerca_del_limite) {
//            ITerm += (ki * error);
//        }
//
//        // Límite del término integral
//        if (ITerm > IoutMax) ITerm = IoutMax;
//        else if (ITerm < IoutMin) ITerm = IoutMin;
//
//        /* Derivada filtrada */
//        double dInput = (input - lastInput) / (timeChange / SampleTime);
//
//        static double _kdContrib_prev = 0;
//        _kdContrib = (-kd * dInput) * (1.0 - D_FILTER_ALFA) + _kdContrib_prev * D_FILTER_ALFA;
//        _kdContrib_prev = _kdContrib;
//
//        /* Salida PID */
//        double output = _kpContrib + ITerm + _kdContrib;
//        double output_pre = output;
//
//        // Saturación de salida
//        if (output > outMax) output = outMax;
//        else if (output < outMin) output = outMin;
//
//        *myOutput = output;
//
//        /* Variables para siguiente ciclo */
//        lastInput = input;
//        lastTime = now;
//
//        /* --- Actualización de clamp_I --- */
//        bool saturated = (output_pre != output);
//        bool equal_sign = (((error > 0 ? 1 : -1) * (output > 0 ? 1 : -1)) == 1);
//
//        // clamp_I activo si estamos saturados y empujando en la misma dirección
//        // o si el actuador aún no ha alcanzado el setpoint
//        clamp_I = ( (saturated && equal_sign) || actuator_busy );
//
//        return true;
//    }
//    else return false;
//}
bool PID_ext::Compute(int rudder_error)
{
    if (!inAuto) return false;

    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);

    static bool timon_en_posicion = false;
    static unsigned long t_timon_estable = 0;
    static double error_prev = 0.0;

    const unsigned long espera_reaccion_ms = 1000; // 1s
    const double error_delta_umbral = 0.2;         // grados

    if (timeChange >= SampleTime)
    {
        /* Entrada y error */
        double input = *myInput;
        double error = *mySetpoint - input;
        _kpContrib = kp * error;

        /* Estado del actuador */
        bool actuator_busy = abs(rudder_error) > 10; // umbral en grados

        // Detección de llegada a posición
        if (!actuator_busy && !timon_en_posicion) {
            timon_en_posicion = true;
            t_timon_estable = now;
            error_prev = error;
        } else if (actuator_busy) {
            timon_en_posicion = false;
        }

        /* Umbral proporcional */
        const double P_THRESHOLD_FACTOR = 0.80;
        double outMaxAbs = fmax(fabs(outMax), fabs(outMin));
        double P_threshold = P_THRESHOLD_FACTOR * outMaxAbs;
        bool P_cerca_del_limite = (fabs(_kpContrib) >= P_threshold);

        /* Sin reacción detectada */
        bool sin_reaccion = fabs(error - error_prev) < error_delta_umbral;

        /* Condición de integración */
        bool permitir_integral = (!clamp_I && !actuator_busy && P_cerca_del_limite);
        if (timon_en_posicion && (now - t_timon_estable) > espera_reaccion_ms && sin_reaccion) {
            permitir_integral = true;
        }

        /* Boost adaptativo */
        if (permitir_integral) {
            double ki_boost = ki;
            if (timon_en_posicion && (now - t_timon_estable) > espera_reaccion_ms && sin_reaccion) {
                double boost_factor = 1.0 + ((now - t_timon_estable) / 3000.0); // +1 cada 3s
                if (boost_factor > 5.0) boost_factor = 5.0; // límite máx
                ki_boost *= boost_factor;
            }
            ITerm += (ki_boost * error);
        }

        /* Límite integral */
        if (ITerm > IoutMax) ITerm = IoutMax;
        else if (ITerm < IoutMin) ITerm = IoutMin;

        /* Derivada filtrada */
        double dInput = (input - lastInput) / (timeChange / SampleTime);
        static double _kdContrib_prev = 0;
        _kdContrib = (-kd * dInput) * (1.0 - D_FILTER_ALFA) + _kdContrib_prev * D_FILTER_ALFA;
        _kdContrib_prev = _kdContrib;

        /* Salida PID */
        double output_pre = _kpContrib + ITerm + _kdContrib;
        double output = output_pre;
        if (output > outMax) output = outMax;
        else if (output < outMin) output = outMin;
        *myOutput = output;

        /* Siguiente ciclo */
        lastInput = input;
        lastTime = now;

        /* Anti-windup */
        bool saturated = (output_pre != output);
        bool equal_sign = (((error > 0 ? 1 : -1) * (output > 0 ? 1 : -1)) == 1);
        clamp_I = ((saturated && equal_sign) || actuator_busy);

        return true;
    }
    return false;
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

