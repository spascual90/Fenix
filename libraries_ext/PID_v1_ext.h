#ifndef PID_v1_ext_h
#define PID_v1_ext_h

#include <PID_v1.h>

// *** PID_v1_ext.h ***
// Limit Max. ITerm contribution
#define I_MAX_COEFICIENT 3.0
//reset I term for changes higher than I_MAX_DELTA
#define I_MAX_DELTA 40.0
// low-pass filter for derivative contribution. ALFA 0: No filter. ALFA 1: All filtered
#define D_FILTER_ALFA 0.8


class PID_ext : public PID {

  public:
  //commonly used functions **************************************************************************
	PID_ext (double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and
		double, double, double, int);     //   Setpoint.  Initial tuning parameters are also set here

    bool Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetTunings(double Kp, double Ki, double Kd);
    void SetOutputLimits(double Min, double Max);
    //MODIFIED SPM
    unsigned long GetSampleTime();
    double getSetpoint();
    //END SPM MODIF
    bool resetITerm(float delta);
    void resetITerm(void);
										  
										  
  //Display functions ****************************************************************
	double getKdContrib();
	double getKpContrib();

  private:
    //SPM INI
    double _kpContrib, _kdContrib;
    //Limit I contribution to PID
    double IoutMin, IoutMax;
    //anti-windup function
    bool clamp_I = false;

    //SPM FIN
};
#endif

