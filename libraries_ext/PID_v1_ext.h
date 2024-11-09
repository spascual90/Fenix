#ifndef PID_v1_ext_h
#define PID_v1_ext_h

#include <PID_v1.h>

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
    //MODIFIED SPM
    unsigned long GetSampleTime();
    double getSetpoint();
    //END SPM MODIF
										  
										  
										  
  //Display functions ****************************************************************
	double getKdContrib();
	double getKpContrib();

  private:
    //SPM INI
    double _kpContrib, _kdContrib;
    //SPM FIN
};
#endif

