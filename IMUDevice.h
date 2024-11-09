/*
 * IMUDevice.h
 *
 *  Created on: 22 ene. 2023
 *      Author: Carmen
 */

#ifndef IMUDEVICE_H_
#define IMUDEVICE_H_
#include <utility/imumaths.h>
#include <EEPROM.h>
//
#include <Wire.h>
#include "I2Cdev.h"

// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

float inline reduce360 (float value) {if (value<0) value+= 360;return fmod (value, 360.0);}

#if defined (BNO055_INTERNAL_FUSION) or defined(BNO055_EXTERNAL_FUSION)
	#define IMUDEVICE_ID 1
	#define IMUDEVICE_NAME "BNO055"
#endif

#ifdef MINIMU9V5
	#define IMUDEVICE_ID 2
	#define IMUDEVICE_NAME "MiniIMU9v5"
#endif

#ifdef ICM20948
	#define IMUDEVICE_ID 4
	#define IMUDEVICE_NAME "ICM20948"
#endif

#ifdef MPU9250
	#define IMUDEVICE_ID 3
	#define IMUDEVICE_NAME "MPU9250"
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
    virtual void IBIT(void);
    virtual bool IMU_setup(long EE_address);
    virtual bool IMU_startCalibration(bool completeCal);
    virtual bool EEload_Calib(long int &eeaddress);
    virtual bool EEsave_Calib(long int &eeaddress);
    virtual void displaySensorOffsets(void);
    virtual float updateHeading();
    virtual bool IMU_Cal_Loop(bool completeCal);
    virtual bool getCalibrationStatus(uint8_t &system, uint8_t &gyro, uint8_t &accel, uint8_t &mag);
    virtual bool IMU_Cal_stopRequest(void);

    //I2C Configuration
    int get_PIN_SDA() {return PIN_SDA;}
    int get_PIN_SCL() {return PIN_SCL;}

    //Device ID
     const int get_IMUdeviceID(void) {return IMUDEVICE_ID;};
     const char *IMUName() { return IMUDEVICE_NAME; }


};


#endif /* IMUDEVICE_H_ */
