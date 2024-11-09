/*
 * DevICM20948.cpp
 *
 *  Created on: 24 ene. 2023
 *      Author: Carmen
 */

#include "DevICM20948.h"


DevICM20948::DevICM20948(void)
{
}

DevICM20948::~DevICM20948() {
	// TODO Auto-generated destructor stub
}

bool DevICM20948::IMU_setup(long EE_address){
	bool orientation;
//	if (IMU_ORIENTATION==0) orientation = true; //Components on top 0 //true: COMPONENTS ON TOP
//	if (IMU_ORIENTATION==1) orientation = false;
	return ICM20948AHRS_setup(orientation);
}

void DevICM20948::IBIT(void){

	DEBUG_print("IMU int... Started\n");
	sprintf(DEBUG_buffer,"HW: %s\nSDA,SCL=%i,%i\n", IMUName(), get_PIN_SDA(),get_PIN_SCL());

	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
}

float DevICM20948::updateHeading(void){
	//return reduce360(ICM20948AHRS_loop());
	return ICM20948AHRS_loop();
}


bool DevICM20948::EEload_Calib(long int &eeAddress)
{
	//  Get and restore Calibration offsets
	long EE_ID, ID;
	// Variables calibración ICM20948
	float G_offset[3];
	float A_B[3];
	float A_Ainv[3][3];
	float M_B[3];
	float M_Ainv[3][3];

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
//		m_min.x = -10000;
//		m_min.y = -10000;
//		m_min.z = -10000;
//		m_max.x =10000;
//		m_max.y =10000;
//		m_max.z =10000;
//		offset_g.x = 0;
//		offset_g.y = 0;
//		offset_g.z = 0;
//		offset_a.x = 0;
//		offset_a.y = 0;
//		offset_a.z = 0;
//		_gyro=0;
//		_accel=0;
//		_mag=0;

		//Gyro default scale 250 dps. Convert to radians/sec subtract offsets
		float lG_offset[3] = {240.7, 227.7, -4.8};
		//Accel scale: divide by 16604.0 to normalize
		float lA_B[3] = { 470.7 , -83.03 , 300.65 };
		float lA_Ainv[3][3] = {
		{ 0.06232 , -0.00413 , -0.00479 },
		{ -0.00413 , 0.0632 , -0.0017 },
		{ -0.00479 , -0.0017 , 0.0589 }};
		//Mag scale divide by 369.4 to normalize
		float lM_B[3] = { -80.39 , -35.59 , 18.54 };
		float lM_Ainv[3][3] = {
		{ 4.47829 , -0.06586 , 0.02004 },
		{ -0.06586 , 4.53222 , 0.00443 },
		{ 0.02004 , 0.00443 , 4.46886 }};
		//ICM20948: mandar valores de variables calibración
		ICM20948AHRS_setOffsets(lG_offset, lA_B, lA_Ainv,  lM_B, lM_Ainv);
		displaySensorOffsets();
		return false;

	} else {
		DEBUG_print("!Found Calibration data...\n");
		_gyro=3;
		_accel=3;
		_mag=3;
//ICM20948: Cargar valores de variables calibración


	eeAddress += sizeof(ID);
	EEPROM.get(eeAddress, G_offset);
	eeAddress += sizeof(G_offset);
	EEPROM.get(eeAddress, A_B);
	eeAddress += sizeof(A_B);
	EEPROM.get(eeAddress, A_Ainv);
	eeAddress += sizeof(A_Ainv);
	EEPROM.get(eeAddress, M_B);
	eeAddress += sizeof(M_B);
	EEPROM.get(eeAddress, M_Ainv);
	}
	// SPM: Force hardcoded calibration values here

	//ICM20948: mandar valores de variables calibración
	ICM20948AHRS_setOffsets(G_offset, A_B, A_Ainv,  M_B, M_Ainv);
	displaySensorOffsets();
	return true;
}
bool DevICM20948::EEsave_Calib( long &eeAddress){


	bool DataStored = false;
 	//DATA TO SAVE
	long ID;
	// Variables calibración ICM20948
	float G_offset[3];
	float A_B[3];
	float A_Ainv[3][3];
	float M_B[3];
	float M_Ainv[3][3];

	DEBUG_print("!Saving Calibration...");

	//ID
    ID = get_IMUdeviceID();
    EEPROM.put(eeAddress, ID);
    eeAddress += sizeof(ID);

    //Calib
    //ICM20948: Recibir valores de variables calibración
    ICM20948AHRS_getOffsets(G_offset, A_B, A_Ainv, M_B,  M_Ainv);
	//ICM20948: Grabar valores de variables calibración
    EEPROM.put(eeAddress, G_offset);
    eeAddress += sizeof(G_offset);
    EEPROM.put(eeAddress, A_B);
    eeAddress += sizeof(A_B);
    EEPROM.put(eeAddress, A_Ainv);
    eeAddress += sizeof(A_Ainv);
    EEPROM.put(eeAddress, M_B);
    eeAddress += sizeof(M_B);
    EEPROM.put(eeAddress, M_Ainv);
    eeAddress += sizeof(M_Ainv);

    DataStored = true;

	DEBUG_print("Ok\n");
    return DataStored;

}

void DevICM20948::displaySensorOffsets(void){
	//ICM20948: Recibir valores de variables calibración e imprimirlos
	int l=8, d=2, d5=5;
	char c1[l+3];
	char c2[l+3];
	char c3[l+3];
	// Variables calibración ICM20948
	float G_offset[3];
	float A_B[3];
	float A_Ainv[3][3];
	float M_B[3];
	float M_Ainv[3][3];

	ICM20948AHRS_getOffsets(G_offset, A_B, A_Ainv, M_B,  M_Ainv);

	sprintf(DEBUG_buffer,"G_offset: x:%s, y:%s, z:%s\n", dtostrf(G_offset[0],l,d,c1), dtostrf(G_offset[1],l,d,c2), dtostrf(G_offset[2],l,d,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
	sprintf(DEBUG_buffer,"A_B: %s \t %s \t %s\n", dtostrf(A_B[0],l,d,c1), dtostrf(A_B[1],l,d,c2), dtostrf(A_B[2],l,d,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
	sprintf(DEBUG_buffer,"A_Ainv:\t %s \t %s \t %s\n", dtostrf(A_Ainv[0][0],l,d5,c1), dtostrf(A_Ainv[0][1],l,d5,c2), dtostrf(A_Ainv[0][2],l,d5,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
	sprintf(DEBUG_buffer,"A_Ainv:\t %s \t %s \t %s\n", dtostrf(A_Ainv[1][0],l,d5,c1), dtostrf(A_Ainv[1][1],l,d5,c2), dtostrf(A_Ainv[1][2],l,d5,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
	sprintf(DEBUG_buffer,"A_Ainv:\t %s \t %s \t %s\n", dtostrf(A_Ainv[2][0],l,d5,c1), dtostrf(A_Ainv[2][1],l,d5,c2), dtostrf(A_Ainv[2][2],l,d5,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
	sprintf(DEBUG_buffer,"M_B: %s \t %s \t %s\n", dtostrf(M_B[0],l,d,c1), dtostrf(M_B[1],l,d,c2), dtostrf(M_B[2],l,d,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
	sprintf(DEBUG_buffer,"M_Ainv: %s \t %s \t %s\n", dtostrf(M_Ainv[0][0],l,d5,c1), dtostrf(M_Ainv[0][1],l,d5,c2), dtostrf(M_Ainv[0][2],l,d5,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
	sprintf(DEBUG_buffer,"M_Ainv: %s \t %s \t %s\n", dtostrf(M_Ainv[1][0],l,d5,c1), dtostrf(M_Ainv[1][1],l,d5,c2), dtostrf(M_Ainv[1][2],l,d5,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
	sprintf(DEBUG_buffer,"M_Ainv: %s \t %s \t %s\n", dtostrf(M_Ainv[2][0],l,d5,c1), dtostrf(M_Ainv[2][1],l,d5,c2), dtostrf(M_Ainv[2][2],l,d5,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
}

bool DevICM20948::IMU_startCalibration(bool completeCal) {
	_gyro = 1;
	_accel = 1;
	// Accelerometer and gyroscope calibration
	DEBUG_print("Gyro and Accel calibration. Keep device stable...");
	_gyro = 3;
	_accel = 3;
	DEBUG_print("Magnetic calibration. Make a figure eight pattern...\n");
	// Magnetic calibration
	_start_cal_time = millis();
	// Calibration just started
	_mag = 1;
	return ICM20948AHRS_calibration_setup();
}


// return false when calibration is finished
// true, calibration ongoing
bool DevICM20948::IMU_Cal_Loop(bool completeCal){
	bool ret = ICM20948AHRS_calibration_loop();
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

bool DevICM20948::getCalibrationStatus(uint8_t &system, uint8_t &gyro, uint8_t &accel, uint8_t &mag){
	gyro=_gyro;
	accel=_accel;
	mag=_mag;
	system=(_gyro+_accel+_mag)/3;
}

bool DevICM20948::IMU_Cal_stopRequest(void) {
	//_iter = _max_iter;
	return true;
}



