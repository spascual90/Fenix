/*
 * DevICM20948.h
 *
 *  Created on: 23 ene. 2023
 *      Author: Carmen
 */

#ifndef DEVICM20948_H_
#define DEVICM20948_H_

#include <utility/imumaths.h>
#include <EEPROM.h>
#include <Wire.h>
#include "I2Cdev.h"

// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"
#include "IMUDevice.h"

#define ACC_MAG_LOOPS 600
#define SENSOR_ACCEL false
#define SENSOR_MAGNET true

//External variables and functions implemented in ICM20948AHRS.cpp and ICM20948Calibrate.cpp
TwoWire* ICM20948AHRS_setup(bool orientation = true); // true: COMPONENTS ON TOP false: COMPONENTS BOTTOM
float ICM20948AHRS_loop(void); //Main Loop
int16_t* ICM20948AHRS_calibration_setup(void);
int16_t* ICM20948AHRS_calibration_loop(bool sensor);

struct s_offset {int16_t x, y, z;};
bool ICM20948AHRS_getOffsets(float aG_offset[3], float aA_B[3], float aA_Ainv[3][3], float aM_B[3], float aM_Ainv[3][3]);
bool ICM20948AHRS_setOffsets(float aG_offset[3], float aA_B[3], float aA_Ainv[3][3], float aM_B[3], float aM_Ainv[3][3]);

class DevICM20948 : public IMUDevice {
public:

	DevICM20948();
	~DevICM20948();

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
    //void print_Values(void);
    void displayRaw_gyroOffsets(int16_t *gyro);
    void displayRaw_acc_magOffsets(int16_t *acc_mag);
    void displayRaw_SumOffsets(int16_t **acc_mag);

private:
    //Magnetometer calibration variables
	//long _start_cal_time = 0;
    const long _max_iter=10000; //Max. milliseconds after last calibration change
    uint8_t _mag=0, _gyro=0, _accel=0;

    int16_t* gyro;
    int _acc_mag_count = ACC_MAG_LOOPS; //collect this many values for acc/mag calibration
	bool _sensor = SENSOR_ACCEL;


};

#endif /* DEVICM20948_H_ */
