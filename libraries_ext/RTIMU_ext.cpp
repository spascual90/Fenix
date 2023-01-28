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

#include "RTIMU_ext.h"
#include "Arduino.h"

RTIMU_ext::RTIMU_ext(RTIMUSettings *settings)
:RTIMU(settings)
{
}

RTIMU_ext::~RTIMU_ext()
{
}

void RTIMU_ext::setCalibrationData(void)
{
    float maxDelta = -1;
    float delta;
    CALLIB_DATA calData;

    m_calibrationValid = false;

	sprintf(DEBUG_buffer,"Load IMU Calib. Address :%i. Lenght: %i\n",_eeAddress, sizeof(calData) );
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();


    if (calLibRead(_eeAddress, &calData)) {
        if (calData.magValid != 1) {
            return;
        }

        //  find biggest range

        for (int i = 0; i < 3; i++) {
            if ((calData.magMax[i] - calData.magMin[i]) > maxDelta)
                maxDelta = calData.magMax[i] - calData.magMin[i];
        }
        if (maxDelta < 0) {
            return;
        }
        maxDelta /= 2.0f;                                       // this is the max +/- range

        for (int i = 0; i < 3; i++) {
            delta = (calData.magMax[i] - calData.magMin[i]) / 2.0f;
            m_compassCalScale[i] = maxDelta / delta;            // makes everything the same range
            m_compassCalOffset[i] = (calData.magMax[i] + calData.magMin[i]) / 2.0f;
        }
        m_calibrationValid = true;
    }
}


