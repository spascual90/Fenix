#ifndef PID_v1_ext_h
#define PID_v1_ext_h

#include <PID_v1.h>

// *** PID_v1_ext.h ***
// Limit Max. ITerm contribution
#define I_MAX_COEFICIENT 1.5
//reset I term for changes higher than I_MAX_DELTA
#define I_MAX_DELTA 40.0
// low-pass filter for derivative contribution. ALFA 0: No filter. ALFA 1: All filtered
constexpr double ALFA_20 = 0.2;
constexpr double ALFA_80 = 0.8;

constexpr double ALFA_10 = 0.1;
constexpr double ALFA_90 = 0.9;

constexpr double MAX_PID_TURN = 100;

#define I_ACTUATOR_BUSY_THRESHOLD 10
#define D_DERR_UMBRAL 0.5  // grados/ segundo. Velocidad media de cambio del error (rumbo) durante el último segundo < se considera que el rumbo es estable
#define D_RUDDER_ERROR_UMBRAL 10.0 // x/10 = 1  grado de timón. < se considera que el timón ha llegado a su angulo objetivo

typedef enum type_DBConfig {MAXDB, MINDB, AUTODB, LOOP} type_DBConfig;

//Deadband constant factors
constexpr double MIN_DEADBAND = 0.5;   // grados mínimos
constexpr double MAX_DEADBAND = 5.0;   // grados máximos
constexpr double FACTOR_DEADBAND = 2.0; // amplificación del ruido


class PID_ext : public PID {

  public:
  //commonly used functions **************************************************************************
	PID_ext (double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and
		double, double, double, int, float);

	     //   Setpoint.  Initial tuning parameters are also set here

    bool Compute(int rudder_error, float speed);                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively
    void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)
    void Initialize (void);
    void SetTunings(double Kp, double Ki, double Kd);
    void SetOutputLimits(double Min, double Max);
    unsigned long GetSampleTime();
    double getSetpoint();
    bool resetITerm(float delta);
    void resetITerm(void);
										  
										  
  //Display functions ****************************************************************
	double getKdContrib();
	double getKpContrib();

	float getSpeed_ref() const {
		return _speed_ref;
	}

	void setSpeed_ref(float speed_ref) {
		_speed_ref = speed_ref;
	}

    double getDeadband(void) const {
    	return _deadband;
    }

	bool isInDeadband() const {
		return _inDeadband;
	}

  protected:
	type_DBConfig getTypeDB() const {
		return _type_DB;
	}

	void setTypeDB(type_DBConfig typeDb) {
		_type_DB = typeDb;
	}

  private:
    //SPM INI
    double _kpContrib, _kdContrib;
    //Limit I contribution to PID
    double IoutMin, IoutMax;
    // speed reference for PID performance
    float _speed_ref = 5.0;                   // velocidad de referencia (valor se define en el constructor)
    double _deadband = MIN_DEADBAND;
    type_DBConfig _type_DB;
    bool _inDeadband = false;
    bool calcDeadband(double error, double error_prev);
    double _dInput_prev = 0;
    double _kdContrib_prev = 0;
};
#endif

