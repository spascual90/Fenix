/*
 * DevMinIMU9V5.h
 *
 *  Created on: 23 ene. 2023
 *      Author: Carmen
 */

#ifndef DEVMINIMU9V5_H_
#define DEVMINIMU9V5_H_

#include <utility/imumaths.h>
#include <EEPROM.h>
#include <Wire.h>
#include "I2Cdev.h"

// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

#include "IMUDevice.h"

#define IMUDEVICE_ID 1

//External variables and functions implemented in MinIMU9AHRS.cpp
void MinIMU9AHRS_setup(void);
float MinIMU9AHRS_loop(void); //Main Loop
bool MinIMU9AHRS_calibration_setup(void);
bool MinIMU9AHRS_calibration_loop(void);
struct s_offset {int16_t x, y, z;};
bool MinIMU9AHRS_getOffsets(s_offset &m_min, s_offset &m_max);
bool MinIMU9AHRS_setOffsets(s_offset &m_min, s_offset &m_max);

extern bool ext_cal_status_gyro;
extern bool ext_cal_status_accel;
extern bool ext_cal_status_mag;

class DevMinIMU9V5 : public IMUDevice {
public:

	DevMinIMU9V5();
	~DevMinIMU9V5();

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

    //Device ID
    const int get_IMUdeviceID(void) {return IMUDEVICE_ID;};

protected:
    const char *IMUName() { return "MiniIMU9v5"; }

private:
	long _start_cal_time = 0;
    const long _max_iter=5000; //Max. milliseconds after last calibration change


};

#endif /* DEVMINIMU9V5_H_ */
