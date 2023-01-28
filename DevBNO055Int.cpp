/*
 * DevBNO055Int.cpp
 *
 *  Created on: 24 ene. 2023
 *      Author: Carmen
 */

#include "DevBNO055Int.h"


DevBNO055Int::DevBNO055Int(void)
{
}

DevBNO055Int::~DevBNO055Int() {
	// TODO Auto-generated destructor stub
}

bool DevBNO055Int::IMU_setup(long EE_address){

    Wire.begin();

    _imu = (RTIMUBNO055 *)RTIMU::createIMU(&_settings);                        // create the imu object

   int errcode;
	if ((errcode = _imu->IMUInit()) < 0) {
		sprintf(DEBUG_buffer,"IMU %s. Error Code: %i\n",_imu->IMUName(), errcode);
		DEBUG_print(DEBUG_buffer);
		DEBUG_PORT.flush();
		return false;
	}

	return true;
}

void DevBNO055Int::IBIT(void){

	DEBUG_print("IMU int... Started\n");
	sprintf(DEBUG_buffer,"HW: %s\nSDA,SCL=%i,%i\n", _imu->IMUName(), get_PIN_SDA(),get_PIN_SCL());

	DEBUG_print("Sensor fusion: ");
	DEBUG_print("BNO055 Internal Fusion Mode\n");

	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
}

bool DevBNO055Int::IMU_startCalibration(bool completeCal) {
	//This driver does not calibrate dynamically
	if (completeCal == false) return false;
	return false;
}

bool DevBNO055Int::EEload_Calib(long int & eeAddress)
{
    if (_imu->getCalibrationValid()) {
    	DEBUG_print("Using compass calibration\n");
    } else {
    	DEBUG_print("Compass calibration data not required\n");
    }
    return true;
}
bool DevBNO055Int::EEsave_Calib( long &eeAddress){
	DEBUG_print("Compass calibration data not required\n");
    return true;
}

void DevBNO055Int::displaySensorOffsets(void){
	DEBUG_print("Sensor offsets are internal.\n");
    return;
}
float DevBNO055Int::updateHeading(void){
   	int loopCount = 1;

    while (_imu->IMURead()) {
    	if (++loopCount >= 10)  // this flushes remaining data in case we are falling behind
    		continue;
    return reduce360(((RTVector3&)_imu->getFusionPose()).z() * RTMATH_RAD_TO_DEGREE);
    }
}

bool DevBNO055Int::IMU_Cal_Loop(bool completeCal){
	//This driver does not calibrate dynamically
	if (completeCal == false) return false;
	return false;
}

bool DevBNO055Int::getCalibrationStatus(uint8_t &system, uint8_t &gyro, uint8_t &accel, uint8_t &mag){
	//TODO: Evaluate internal calibration status
	system= 3;
	gyro = 3;
	accel = 3;
	mag = 3;
	return true;
}

bool DevBNO055Int::IMU_Cal_stopRequest(void) {
	return true;
}
