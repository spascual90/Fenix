/*
 * DevBNO055Ext.cpp
 *
 *  Created on: 24 ene. 2023
 *      Author: Carmen
 */

#include "DevBNO055Ext.h"


DevBNO055Ext::DevBNO055Ext(void)
{
}

DevBNO055Ext::~DevBNO055Ext() {
	// TODO Auto-generated destructor stub
}

bool DevBNO055Ext::IMU_setup(long EE_address){

    Wire.begin();

    _imu = (RTIMU *)RTIMU::createIMU(&_settings);                        // create the imu object

   int errcode;
	if ((errcode = _imu->IMUInit()) < 0) {
		sprintf(DEBUG_buffer,"IMU %s. Error Code: %i\n",_imu->IMUName(), errcode);
		DEBUG_print(DEBUG_buffer);
		DEBUG_PORT.flush();
		return false;
	}

    // Slerp power controls the fusion and can be between 0 and 1
    // 0 means that only gyros are used, 1 means that only accels/compass are used
    // In-between gives the fusion mix.

    _fusion.setSlerpPower(0.02);

    // use of sensors in the fusion algorithm can be controlled here
    // change any of these to false to disable that sensor

    _fusion.setGyroEnable(true);
    _fusion.setAccelEnable(true);
    _fusion.setCompassEnable(true);

	return true;
}

void DevBNO055Ext::IBIT(void){

	DEBUG_print("IMU int... Started\n");
	sprintf(DEBUG_buffer,"HW: %s\nSDA,SCL=%i,%i\n", _imu->IMUName(), get_PIN_SDA(),get_PIN_SCL());

	DEBUG_print("Sensor fusion: ");
	DEBUG_print("BNO055 External Fusion Mode\n");

	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
}

bool DevBNO055Ext::IMU_startCalibration(bool completeCal) {
	//This driver does not calibrate dynamically
	if (completeCal == false) return false;
	return false;
}

bool DevBNO055Ext::EEload_Calib(long int & eeAddress)
{
    if (_imu->getCalibrationValid()) {
    	DEBUG_print("Using compass calibration\n");
    } else {
    	DEBUG_print("Compass calibration data not required\n");
    }
    return true;
}
bool DevBNO055Ext::EEsave_Calib( long &eeAddress){
	DEBUG_print("Compass calibration data not required\n");
    return true;
}

void DevBNO055Ext::displaySensorOffsets(void){
	DEBUG_print("Sensor offsets are internal.\n");
    return;
}
float DevBNO055Ext::updateHeading(void){
   	int loopCount = 1;

    while (_imu->IMURead()) {
    	if (++loopCount >= 10)  // this flushes remaining data in case we are falling behind
    		continue;
        _fusion.newIMUData(_imu->getGyro(), _imu->getAccel(), _imu->getCompass(), _imu->getTimestamp());

//        if (_imu->IMUGyroBiasValid())
//        	DEBUG_print("Gyro bias valid\n");
//        else
//        	DEBUG_print("...calculating gyro bias\n");

        return reduce360(((RTVector3&)_fusion.getFusionPose()).z() * RTMATH_RAD_TO_DEGREE);
    }
}

bool DevBNO055Ext::IMU_Cal_Loop(bool completeCal){
	//This driver does not calibrate dynamically
	if (completeCal == false) return false;
	return false;
}

bool DevBNO055Ext::getCalibrationStatus(uint8_t &system, uint8_t &gyro, uint8_t &accel, uint8_t &mag){
	//TODO: Evaluate internal calibration status
	system= 3;
	gyro = 3;
	accel = 3;
	mag = 3;
	return true;
}

bool DevBNO055Ext::IMU_Cal_stopRequest(void) {
	return true;
}
