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
		//sprintf(DEBUG_buffer,"IMU %s. Error Code: %i\n",_imu->IMUName(), errcode);
		////DEBUG_print(DEBUG_buffer);
		//DEBUG_flush();
		return false;
	}

	return true;
}

void DevBNO055Int::IBIT(void){

	////DEBUG_print(F("IMU int... Started\n"));
	//sprintf(DEBUG_buffer,"HW: %s\nSDA,SCL=%i,%i\n", IMUName(), get_PIN_SDA(),get_PIN_SCL());
	////DEBUG_print(F("Internal Fusion Mode\n"));

	////DEBUG_print(DEBUG_buffer);
	//DEBUG_flush();
}

float DevBNO055Int::updateHeading(void){
   	int loopCount = 1;

    while (_imu->IMURead()) {
    	if (++loopCount >= 10)  // this flushes remaining data in case we are falling behind
    		continue;
    return reduce360(((RTVector3&)_imu->getFusionPose()).z() * RTMATH_RAD_TO_DEGREE);
    }
}

bool DevBNO055Int::EEload_Calib(long int & eeAddress)
{
	//  Get and restore BNO Calibration offsets
	long EE_ID, ID;
	adafruit_bno055_offsets_t calibrationData;
	//  Look for the sensor's unique ID in EEPROM.
	EEPROM.get(eeAddress, EE_ID);
	// Look for unique ID reported by IMU
	ID = get_IMUdeviceID();
	////sprintf(DEBUG_buffer,"IMU Sensor ID: %i\n",ID);
	//////DEBUG_print(DEBUG_buffer);

	if (EE_ID != ID) {
		////DEBUG_print(F("!WARNING: No Calibration Data for this sensor found!\n"));
		////sprintf(DEBUG_buffer,"ID found: %i\n",EE_ID);
		////DEBUG_print(DEBUG_buffer);

		return false;
	}
	////DEBUG_print(F("!Found Calibration data..."));
	eeAddress += sizeof(ID);
	EEPROM.get(eeAddress, calibrationData);
	//displaySensorOffsets(calibrationData);

	if (_imu->setSensorOffsets(calibrationData)){
		////DEBUG_print(F("Ok\n"));
	} else {
		////DEBUG_print(F("Error. Not restored\n"));
	}

	//displaySensorOffsets();

	return true;
}
bool DevBNO055Int::EEsave_Calib( long &eeAddress){
	bool DataStored = false;
 	//DATA TO SAVE
	long ID;
	adafruit_bno055_offsets_t Calib;

	//////DEBUG_print(F("!Saving Calibration..."));

	//ID
    ID = get_IMUdeviceID();
    EEPROM.put(eeAddress, ID);
    eeAddress += sizeof(ID);

    //Calib
	if (_imu->getSensorOffsets(Calib)) {
		EEPROM.put(eeAddress, Calib);
	    DataStored = true;
		////DEBUG_print(F("Ok\n"));
	} else {
		////DEBUG_print(F("Error. Not saved\n"));
	}
    return DataStored;
}

bool DevBNO055Int::IMU_startCalibration(bool completeCal) {
	//IMU calibrates dynamically, no need to launch calibration process
	if (completeCal == false) return false;
	return false;
}

void DevBNO055Int::displaySensorOffsets(void){
	//////DEBUG_print(F("Sensor offsets are internal.\n"));
    return;
}

bool DevBNO055Int::IMU_Cal_Loop(bool completeCal){
	//IMU calibration is internal
	if (completeCal == false) return false;
	return false;
}

bool DevBNO055Int::getCalibrationStatus(uint8_t &system, uint8_t &gyro, uint8_t &accel, uint8_t &mag){
	_imu->getCalibration(&system, &gyro, &accel, &mag);
	return true;
}

bool DevBNO055Int::IMU_Cal_stopRequest(void) {
	return true;
}
