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

//External variables and functions implemented in MinIMU9AHRS.cpp
void MinIMU9AHRS_setup(bool orientation = true); // true: COMPONENTS ON TOP false: COMPONENTS BOTTOM
float MinIMU9AHRS_loop(void); //Main Loop
bool MinIMU9AHRS_calibration_setup(void);
bool MinIMU9AHRS_calibration_loop(void);
bool MinIMU9AHRS_ag_calibration_loop(void);

struct s_offset {int16_t x, y, z;};
bool MinIMU9AHRS_getOffsets(s_offset &m_min, s_offset &m_max, s_offset &ag_offset, s_offset &aa_offset);
bool MinIMU9AHRS_setOffsets(s_offset &m_min, s_offset &m_max, s_offset &ag_offset, s_offset &aa_offset);

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

private:
    //Magnetometer calibration variables
	long _start_cal_time = 0;
    const long _max_iter=10000; //Max. milliseconds after last calibration change
    uint8_t _mag=0, _gyro=0, _accel=0;


};

#endif /* DEVMINIMU9V5_H_ */
