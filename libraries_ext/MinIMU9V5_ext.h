////////////////////////////////////////////////////////////////////////////
#ifndef _MINIMU9V5_EXT_H
#define	_MINIMU9V5_EXT_H

#include <RTIMU.h>
#include "CalLib.h"
#include "GPSport.h" // Serial NMEA IF Configuration in GPSPort.h not in Fenix.ino!

class MINIMU9V5_ext
{
public:
    //  Constructor/destructor

	MINIMU9V5_ext();
    virtual ~MINIMU9V5_ext();

    //  setCalibrationData configured the cal data and also enables use if valid

    void setCalibrationData();

	long getEeAddress() const {
		return _eeAddress;
	}

	void setEeAddress(long eeAddress = 0) {
		_eeAddress = eeAddress;
	}

private:
    long _eeAddress=0;
 };

#endif // _MINIMU9V5_EXT_H
