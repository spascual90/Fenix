/*
 * DevBNO055Ext.cpp
 *
 *  Created on: 24 ene. 2023
 *      Author: Carmen
 */

#include "DevBNO055Ext.h"
#include "GPSport.h" // uncomment this line to print debug messages to serial monitor
#include <simplot.h> //SIMPLOT FOR DEBUGGING PURPOSE ONLY


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
		DEBUG_flush();
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

	DEBUG_print(F("IMU int... Started\n"));
	sprintf(DEBUG_buffer,"HW: %s\nSDA,SCL=%i,%i\n", _imu->IMUName(), get_PIN_SDA(),get_PIN_SCL());

	DEBUG_print(F("Sensor fusion: "));
	DEBUG_print(F("BNO055 External Fusion Mode\n"));

	DEBUG_print(DEBUG_buffer);
	// SPM ELIMINADO DEBUG_PORT.flush();
}


float DevBNO055Ext::updateHeading(void){
   	int loopCount = 1;

    while (_imu->IMURead()) {
    	if (++loopCount >= 10) {
    		// this flushes remaining data in case we are falling behind
    		DEBUG_print(F("!IMU Timeout\n"));
    		continue;

    	}
        _fusion.newIMUData(_imu->getGyro(), _imu->getAccel(), _imu->getCompass(), _imu->getTimestamp());

//        if (_imu->IMUGyroBiasValid())
//        	DEBUG_print(F("Gyro bias valid\n"));
//        else
//        	DEBUG_print(F("...calculating gyro bias\n"));
        //plot2(NeoSerial,int(((RTVector3&)_fusion.getFusionPose()).z()* RTMATH_RAD_TO_DEGREE),int(_imu->DOF_x* RTMATH_RAD_TO_DEGREE));
        //plot1(NeoSerial,int(((RTVector3&)_fusion.getFusionPose()).z()* RTMATH_RAD_TO_DEGREE));

        		//int(((RTVector3&)_fusion.getFusionPose()).x()*1000), int(((RTVector3&)_fusion.getFusionPose()).y()*1000), int(((RTVector3&)_fusion.getFusionPose()).z()*1000));

        return reduce360(((RTVector3&)_fusion.getFusionPose()).z() * RTMATH_RAD_TO_DEGREE);
    }
}
bool DevBNO055Ext::EEload_Calib(long int & eeAddress)
{
	//  Get and restore BNO Calibration offsets
	long EE_ID, ID;
	adafruit_bno055_offsets_t calibrationData;
	//  Look for the sensor's unique ID in EEPROM.
	EEPROM.get(eeAddress, EE_ID);
	// Look for unique ID reported by IMU
	ID = get_IMUdeviceID();
	sprintf(DEBUG_buffer,"IMU Sensor ID: %i\n",ID);
	DEBUG_print(DEBUG_buffer);

	if (EE_ID != ID) {
		DEBUG_print(F("!WARNING: No Calibration Data for this sensor found!\n"));
		sprintf(DEBUG_buffer,"ID found: %i\n",EE_ID);
		DEBUG_print(DEBUG_buffer);

		return false;
	}
	DEBUG_print(F("!Found Calibration data..."));
	eeAddress += sizeof(ID);
	EEPROM.get(eeAddress, calibrationData);
	//displaySensorOffsets(calibrationData);

	if (_imu->setSensorOffsets(calibrationData)){
		DEBUG_print(F("Ok\n"));
	} else {
		DEBUG_print(F("Error. Not restored\n"));
	}

	//displaySensorOffsets();

	return true;
}
bool DevBNO055Ext::EEsave_Calib( long &eeAddress){
	bool DataStored = false;
 	//DATA TO SAVE
	long ID;
	adafruit_bno055_offsets_t Calib;

	DEBUG_print(F("!Saving Calibration..."));

	//ID
    ID = get_IMUdeviceID();
    EEPROM.put(eeAddress, ID);
    eeAddress += sizeof(ID);

    //Calib
	if (_imu->getSensorOffsets(Calib)) {
		EEPROM.put(eeAddress, Calib);
	    DataStored = true;
		DEBUG_print(F("Ok\n"));
	} else {
		DEBUG_print(F("Error. Not saved\n"));
	}
    return DataStored;
}

bool DevBNO055Ext::IMU_startCalibration(bool completeCal) {
	//IMU calibrates dynamically, no need to launch calibration process
	if (completeCal == false) return false;
	return false;
}

void DevBNO055Ext::displaySensorOffsets(void){
	DEBUG_print(F("Sensor offsets are internal.\n"));
    return;
}
bool DevBNO055Ext::IMU_Cal_Loop(bool completeCal){
	//This driver does not calibrate dynamically
	if (completeCal == false) return false;
	return false;
}

bool DevBNO055Ext::getCalibrationStatus(uint8_t &system, uint8_t &gyro, uint8_t &accel, uint8_t &mag){
	_imu->getCalibration(&system, &gyro, &accel, &mag);
	return true;
}

bool DevBNO055Ext::IMU_Cal_stopRequest(void) {
	return true;
}
