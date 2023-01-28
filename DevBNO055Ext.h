/*
 * DevBNO055Ext.h
 *
 *  Created on: 23 ene. 2023
 *      Author: Carmen
 */

#ifndef DEVBNO055EXT_H_
#define DEVBNO055EXT_H_

#include <utility/imumaths.h>
#include <EEPROM.h>
#include <Wire.h>
#include "I2Cdev.h"

// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

#include "IMUDevice.h"

//RTIMU Lib headers
#include "RTIMU.h"
#include "RTIMUSettings.h"
#include "RTFusionRTQF.h"
#include "CalLib.h"

class DevBNO055Ext : public IMUDevice {
public:

	DevBNO055Ext();
	~DevBNO055Ext();

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

protected:

private:
    RTIMU *_imu;                                           // the IMU object
    RTFusionRTQF _fusion;                                  // the fusion object
    RTIMUSettings _settings;                               // the settings object
};

#endif /* DEVBNO055EXT_H_ */
