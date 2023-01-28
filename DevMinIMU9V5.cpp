/*
 * DevMinIMU9V5.cpp
 *
 *  Created on: 24 ene. 2023
 *      Author: Carmen
 */

#include "DevMinIMU9V5.h"


DevMinIMU9V5::DevMinIMU9V5(void)
{
}

DevMinIMU9V5::~DevMinIMU9V5() {
	// TODO Auto-generated destructor stub
}

bool DevMinIMU9V5::IMU_setup(long EE_address){
	MinIMU9AHRS_setup();
	return true;
}

void DevMinIMU9V5::IBIT(void){

	DEBUG_print("IMU int... Started\n");
	sprintf(DEBUG_buffer,"HW: %s\nSDA,SCL=%i,%i\n", IMUName(), get_PIN_SDA(),get_PIN_SCL());

	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
}

float DevMinIMU9V5::updateHeading(void){
	return reduce360(MinIMU9AHRS_loop());
}


bool DevMinIMU9V5::EEload_Calib(long int &eeAddress)
{
	//  Get and restore Calibration offsets
	long EE_ID, ID;
	struct s_offset m_min, m_max;
	//  Look for the sensor's unique ID in EEPROM.
	EEPROM.get(eeAddress, EE_ID);
	// Look for unique ID reported by IMU
	ID = get_IMUdeviceID();
	sprintf(DEBUG_buffer,"IMU Sensor ID: %i\n",ID);
	DEBUG_print(DEBUG_buffer);

	if (EE_ID != ID) {
		DEBUG_print("!WARNING: No Calibration Data for this sensor found!\n");
		sprintf(DEBUG_buffer,"ID found: %i\n",EE_ID);
		DEBUG_print(DEBUG_buffer);

		return false;
	}
	DEBUG_print("!Found Calibration data...\n");
	eeAddress += sizeof(ID);
	EEPROM.get(eeAddress, m_min);
	eeAddress += sizeof(m_min);
	EEPROM.get(eeAddress, m_max);

	MinIMU9AHRS_setOffsets(m_min, m_max);

	displaySensorOffsets();
	ext_cal_status_mag=true;
	return true;

}
bool DevMinIMU9V5::EEsave_Calib( long &eeAddress){
	struct s_offset m_min, m_max;
	bool DataStored = false;
 	//DATA TO SAVE
	long ID;

	DEBUG_print("!Saving Calibration...");

	//ID
    ID = get_IMUdeviceID();
    EEPROM.put(eeAddress, ID);
    eeAddress += sizeof(ID);

    //Calib
	MinIMU9AHRS_getOffsets(m_min, m_max);

    EEPROM.put(eeAddress, m_min);
    eeAddress += sizeof(m_min);
    EEPROM.put(eeAddress, m_max);
    eeAddress += sizeof(m_max);

    DataStored = true;

	DEBUG_print("Ok\n");
    return DataStored;

}

void DevMinIMU9V5::displaySensorOffsets(void){
	struct s_offset m_min, m_max;
	MinIMU9AHRS_getOffsets( m_min, m_max);

	sprintf(DEBUG_buffer,"Mag_x: min:%i, max:%i\n", m_min.x, m_max.x);
	DEBUG_print(DEBUG_buffer);
	sprintf(DEBUG_buffer,"Mag_y: min:%i, max:%i\n", m_min.y, m_max.y);
	DEBUG_print(DEBUG_buffer);
	sprintf(DEBUG_buffer,"Mag_z: min:%i, max:%i\n", m_min.z, m_max.z);
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
}

bool DevMinIMU9V5::IMU_startCalibration(bool completeCal) {
	_start_cal_time = millis();
	return MinIMU9AHRS_calibration_setup();
}


// return false when calibration is finished
// true, calibration ongoing
bool DevMinIMU9V5::IMU_Cal_Loop(bool completeCal){
	bool ret = MinIMU9AHRS_calibration_loop();
	long current_loop = millis();
	if (ret) {
		_start_cal_time = current_loop;
		DEBUG_print("!Offsets Changed\n");
	}
	return ((current_loop-_start_cal_time)  < _max_iter);
}

bool DevMinIMU9V5::getCalibrationStatus(uint8_t &system, uint8_t &gyro, uint8_t &accel, uint8_t &mag){
}

bool DevMinIMU9V5::IMU_Cal_stopRequest(void) {
	//_iter = _max_iter;
	return true;
}

