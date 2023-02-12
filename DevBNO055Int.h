/*
 * DevBNO055Int.h
 *
 *  Created on: 23 ene. 2023
 *      Author: Carmen
 */

#ifndef DEVBNO055INT_H_
#define DEVBNO055INT_H_

#include <utility/imumaths.h>
#include <EEPROM.h>
#include <Wire.h>
#include "I2Cdev.h"

// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

#include "IMUDevice.h"

//RTIMU Lib headers
#include "RTIMUBNO055.h"
#include "RTIMUSettings.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"

//#define MAX_ITER 100000 // Max number of iterations to consider calibration procedure has failed
//#define CAL_CHECK_LOOP 7000//20000 // Number of iterations to check IMU Calibration results
//#define MAX_LOW_QDATA 100 // Maximum iterations with low quality data from IMU

class DevBNO055Int : public IMUDevice {
public:

	DevBNO055Int();
	~DevBNO055Int();

    void IBIT(void);
    bool IMU_setup(long EE_address);
    bool IMU_startCalibration(bool completeCal);
    bool EEload_Calib(long int &eeaddress);
    bool EEsave_Calib(long int &eeaddress);
    void displaySensorOffsets(void);
    float updateHeading();
    bool IMU_Cal_Loop(bool completeCal);
    bool getCalibrationStatus(uint8_t &system, uint8_t &gyro, uint8_t &accel, uint8_t &mag);
    bool IMU_Cal_stopRequest(void);

private:
	RTIMUBNO055 *_imu;
	RTIMUSettings _settings;                               // the settings object
};

#endif /* DEVBNO055INT_H_ */
