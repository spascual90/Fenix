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
	bool orientation;
	if (IMU_ORIENTATION==0) orientation = true; //Components on top 0 //true: COMPONENTS ON TOP
	if (IMU_ORIENTATION==1) orientation = false;
	return MinIMU9AHRS_setup(orientation);
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
	struct s_offset m_min, m_max, offset_g, offset_a;

	_mag=0;
	_gyro=0;
	_accel=0;

	//  Look for the sensor's unique ID in EEPROM.
	EEPROM.get(eeAddress, EE_ID);
	// Look for unique ID reported by IMU
	ID = get_IMUdeviceID();
	sprintf(DEBUG_buffer,"IMU Sensor ID: %i\n",ID);
	DEBUG_print(DEBUG_buffer);

	if (EE_ID != ID) {
		DEBUG_print("!WARNING: No Calibration Data for this sensor found!\n");
		DEBUG_print("!WARNING: Using default values!\n");
		sprintf(DEBUG_buffer,"ID found: %i\n",EE_ID);
		DEBUG_print(DEBUG_buffer);
		m_min.x = -10000;
		m_min.y = -10000;
		m_min.z = -10000;
		m_max.x =10000;
		m_max.y =10000;
		m_max.z =10000;
		offset_g.x = 0;
		offset_g.y = 0;
		offset_g.z = 0;
		offset_a.x = 0;
		offset_a.y = 0;
		offset_a.z = 0;
		_gyro=0;
		_accel=0;
		_mag=0;

//		return false;
	} else {
		DEBUG_print("!Found Calibration data...\n");
		_gyro=3;
		_accel=3;
		_mag=3;

	eeAddress += sizeof(ID);
	EEPROM.get(eeAddress, m_min);
	eeAddress += sizeof(m_min);
	EEPROM.get(eeAddress, m_max);
	eeAddress += sizeof(m_max);
	EEPROM.get(eeAddress, offset_g);
	eeAddress += sizeof(offset_g);
	EEPROM.get(eeAddress, offset_a);
	}
	// SPM: Force hardcoded calibration values
//	m_min.x = -3918;
//	m_min.y = -3760;
//	m_min.z = -4319;
//	m_max.x =2968;
//	m_max.y =2922;
//	m_max.z =2443;
//	offset_g.x = 17;
//	offset_g.y = -33;
//	offset_g.z = -61;
//	offset_a.x = -35;
//	offset_a.y = 5;
//	offset_a.z = 2;

	MinIMU9AHRS_setOffsets(m_min, m_max, offset_g, offset_a);

	displaySensorOffsets();

	return true;

}
bool DevMinIMU9V5::EEsave_Calib( long &eeAddress){
	struct s_offset m_min, m_max, offset_g, offset_a;
	bool DataStored = false;
 	//DATA TO SAVE
	long ID;

	DEBUG_print("!Saving Calibration...");

	//ID
    ID = get_IMUdeviceID();
    EEPROM.put(eeAddress, ID);
    eeAddress += sizeof(ID);

    //Calib
	MinIMU9AHRS_getOffsets(m_min, m_max, offset_g, offset_a);

    EEPROM.put(eeAddress, m_min);
    eeAddress += sizeof(m_min);
    EEPROM.put(eeAddress, m_max);
    eeAddress += sizeof(m_max);
    EEPROM.put(eeAddress, offset_g);
    eeAddress += sizeof(offset_g);
    EEPROM.put(eeAddress, offset_a);
    eeAddress += sizeof(offset_a);

    DataStored = true;

	DEBUG_print("Ok\n");
    return DataStored;

}

void DevMinIMU9V5::displaySensorOffsets(void){
	struct s_offset m_min, m_max, offset_g, offset_a;
	MinIMU9AHRS_getOffsets( m_min, m_max, offset_g, offset_a);

	sprintf(DEBUG_buffer,"Mag_x: min:%i, max:%i\n", m_min.x, m_max.x);
	DEBUG_print(DEBUG_buffer);
	sprintf(DEBUG_buffer,"Mag_y: min:%i, max:%i\n", m_min.y, m_max.y);
	DEBUG_print(DEBUG_buffer);
	sprintf(DEBUG_buffer,"Mag_z: min:%i, max:%i\n", m_min.z, m_max.z);
	DEBUG_print(DEBUG_buffer);
	sprintf(DEBUG_buffer,"Gyro Offsets: x:%i, y:%i, z:%i\n",offset_g.x,offset_g.y,offset_g.z);
	DEBUG_print();
	sprintf(DEBUG_buffer,"Accel Offsets: x:%i, y:%i, z:%i\n",offset_a.x,offset_a.y,offset_a.z);
	DEBUG_print();
	DEBUG_PORT.flush();
}

bool DevMinIMU9V5::IMU_startCalibration(bool completeCal) {
	_gyro = 1;
	_accel = 1;
	// Accelerometer and gyroscope calibration
	DEBUG_print("Gyro and Accel calibration. Keep device stable...");
	while (!MinIMU9AHRS_ag_calibration_loop()) {
		DEBUG_print("\n!Failed. Keep device stable...");
		delay(1000);
	}
	_gyro = 3;
	_accel = 3;
	DEBUG_print("Magnetic calibration. Make a figure eight pattern...\n");

	// Magnetic calibration
	_start_cal_time = millis();
	// Calibration just started
	_mag = 1;
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
		// Calibration ongoing, new min/MAX values detected
		_mag = 1;
	} else {
		// No new min/MAX value detected
		_mag = 2;
	}

	// Long period without new min/MAX values?
	ret = ((current_loop-_start_cal_time)  < _max_iter);
	// Yes: Magnetometer if fully calibrated!
	if (!ret) _mag = 3;

	return ret;
}

bool DevMinIMU9V5::getCalibrationStatus(uint8_t &system, uint8_t &gyro, uint8_t &accel, uint8_t &mag){
	gyro=_gyro;
	accel=_accel;
	mag=_mag;
	system=(_gyro+_accel+_mag)/3;
}

bool DevMinIMU9V5::IMU_Cal_stopRequest(void) {
	//_iter = _max_iter;
	return true;
}



