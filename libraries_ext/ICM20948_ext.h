////////////////////////////////////////////////////////////////////////////
#ifndef _ICM20948_EXT_H
#define	_ICM20948_EXT_H

#include <RTIMU.h>
#include "CalLib.h"
#include "GPSport.h" // Serial NMEA IF Configuration in GPSPort.h not in Fenix.ino!

class ICM20948_ext
{
public:
    //  Constructor/destructor

	ICM20948_ext();
    virtual ~ICM20948_ext();

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

#endif // _ICM20948_EXT_H
