////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef _RTIMU_EXT_H
#define	_RTIMU_EXT_H

#include <RTIMU.h>
#include "CalLib.h"
#include "GPSport.h" // Serial NMEA IF Configuration in GPSPort.h not in Fenix.ino!

#define I2CWrite(x, y, z) I2Cdev::writeByte(x, y, z)
#define I2CRead(w, x, y, z) I2Cdev::readBytes(w, x, y, z)

class RTIMUSettings;

class RTIMU_ext:public RTIMU
{
public:
    //  Constructor/destructor

    RTIMU_ext(RTIMUSettings *settings);
    virtual ~RTIMU_ext();

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

#endif // _RTIMU_EXT_H
