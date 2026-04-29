/*
 * IMUDevice.h
 *
 *  Created on: 22 ene. 2023
 *      Author: Carmen
 */

#ifndef IMUDEVICE_H_
#define IMUDEVICE_H_
//#include <utility/imumaths.h>
#include <Arduino.h>

#include <EEPROM.h>
#include <Wire.h>

// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

inline float reduce360 (float value) {
	value = fmod (value, 360.0f);
	if (value<0) value+= 360.0f;
	return value;
}

inline float reduce180(float value) {
    // Normalizar primero al rango [0, 360)
	value = fmod (value, 360.0f);
	if (value<0) value+= 360.0f;
    // Convertir a rango [-180, 180)
    if (value >= 180.0f) value -= 360.0f;

    return value;
}

#ifdef MINIMU9V5
	#define IMUDEVICE_ID 2
	#define IMUDEVICE_NAME "MiniIMU9v5"
#endif

#ifdef ICM20948
	#define IMUDEVICE_ID 4
	#define IMUDEVICE_NAME "ICM20948"
#endif

class IMUDevice
{
public:
    //  IMUs should always be created with the following call

    static IMUDevice *createIMUDevice(void);

    //  Constructor/destructor

    IMUDevice();
    virtual ~IMUDevice();

    // CLASS INTERFACE DEFINITION
    //  These functions must be provided by sub classes
    virtual void IBIT(void) = 0;
    virtual bool IMU_setup(long EE_address) = 0;
    virtual bool IMU_startCalibration(char sensor) = 0;
    virtual void Cal_NextSensor(void) = 0;
    virtual bool isExternalCalibration(void) = 0;
    virtual bool EEload_Calib(long int &eeaddress) = 0;
    virtual bool EEsave_Calib(long int &eeaddress) = 0;
    virtual void displaySensorOffsets(void) = 0;
    virtual float updateHeading() = 0;
    virtual float predictYawDelta(float dt) = 0;
    virtual bool IMU_Cal_Loop(void) = 0;
    virtual bool getCalibrationStatus(uint8_t &system, uint8_t &gyro, uint8_t &accel, uint8_t &mag) = 0;
    virtual bool IMU_Cal_stopRequest(void) = 0;
    virtual bool set_calibrate_py_offsets(float B[3], float Ainv[3][3], char sensor) = 0;

    //virtual float get_filtered_psi_dot(void);
    //virtual float get_yaw_accel (void);

    //I2C Configuration
    int get_PIN_SDA() {return SDA;}
    int get_PIN_SCL() {return SCL;}

    //Device ID
     const int get_IMUdeviceID(void) {return IMUDEVICE_ID;};
     const char *IMUName() { return IMUDEVICE_NAME; }


};


#endif /* IMUDEVICE_H_ */
