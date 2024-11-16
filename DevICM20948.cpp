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

		//Gyro default scale 250 dps. Convert to radians/sec subtract offsets
		float lG_offset[3] = {240.7, 227.7, -4.8};
		//Accel scale: divide by 16604.0 to normalize
//		float lA_B[3] = { 470.7 , -83.03 , 300.65 };
//		float lA_Ainv[3][3] = {
//		{ 0.06232 , -0.00413 , -0.00479 },
//		{ -0.00413 , 0.0632 , -0.0017 },
//		{ -0.00479 , -0.0017 , 0.0589 }};
		float lA_B[3] = { -121.55 , -7.98 , 26.18 };
		float lA_Ainv[3][3] = {
		{ 3.21418 , -0.0725 , 0.016 },
		{ -0.0725 , 3.18079 , -0.01859 },
		{ 0.016 , -0.01859 , 3.16865 }};

		//Mag scale divide by 369.4 to normalize
//		float lM_B[3] = { -80.39 , -35.59 , 18.54 };
//		float lM_Ainv[3][3] = {
//		{ 4.47829 , -0.06586 , 0.02004 },
//		{ -0.06586 , 4.53222 , 0.00443 },
//		{ 0.02004 , 0.00443 , 4.46886 }};
		float lM_B[3] = { -117.42 , 7.33 , 34.17 };


		float lM_Ainv[3][3] = {
		{ 3.18336 , -0.04678 , -0.0167 },
		{ -0.04678 , 3.18373 , -0.03753 },
		{ -0.0167 , -0.03753 , 3.13766 }};
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

// CALIBRATION FUNCTIONS

bool DevICM20948::IMU_startCalibration(bool completeCal) {
	_gyro = 1;
	_accel = 1;
	_mag = 1;
	// Accelerometer and gyroscope calibration
	DEBUG_print("Gyro calibration. Keep device stable...\n");
	gyro=ICM20948AHRS_calibration_setup();
	displayRaw_gyroOffsets(gyro);


	DEBUG_print("Accel Magnetic calibration. Make a figure eight pattern...\n");
	DEBUG_print("Raw accelerometer data:\n");
	// Magnetic calibration
	//Restart counter
	_acc_mag_count = ACC_MAG_LOOPS;
	_sensor = SENSOR_ACCEL;
	return true;
}


// return false when calibration is finished
// true, calibration ongoing
bool DevICM20948::IMU_Cal_Loop(bool completeCal){
	bool ret= true;
    int16_t* acc_mag;
	static int cal_loop_temp =0;
	if (cal_loop_temp++ <100) return true;
	cal_loop_temp =0;
	acc_mag = ICM20948AHRS_calibration_loop(_sensor);
	displayRaw_acc_magOffsets(acc_mag);
	_acc_mag_count--;
	if (_acc_mag_count==0) {
		if (_sensor == SENSOR_ACCEL) {
			DEBUG_print("Raw magnetic data:\n");
			_sensor=SENSOR_MAGNET;
			_acc_mag_count=ACC_MAG_LOOPS;
		} else {
			DEBUG_print("Finished.\n");
			ret = false;
		}
	}

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

void DevICM20948::displayRaw_gyroOffsets(int16_t *gyro) {
	sprintf(DEBUG_buffer,"Gyro offsets: %i \t %i \t %i\n", gyro[0],gyro[1],gyro[2]);
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
  }

void DevICM20948::displayRaw_acc_magOffsets(int16_t *acc_mag) {
	sprintf(DEBUG_buffer,"%i,%i,%i\n",acc_mag[0],acc_mag[1],acc_mag[2]);
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
  }

void DevICM20948::displayRaw_SumOffsets(int16_t **acc_mag) {
	int i=0;
	DEBUG_print("Accelerometer raw data:\n");
	for (i=0; i<ACC_MAG_LOOPS; i++){
		sprintf(DEBUG_buffer,"%i,%i,%i\n",acc_mag[i][0],acc_mag[i][1],acc_mag[i][2]);
		DEBUG_print(DEBUG_buffer);
		DEBUG_PORT.flush();
	}

	DEBUG_print("Magnet raw data:\n");
	for (i=0; i<ACC_MAG_LOOPS; i++){
		sprintf(DEBUG_buffer,"%i,%i,%i\n",acc_mag[i][3],acc_mag[i][4],acc_mag[i][5]);
		DEBUG_print(DEBUG_buffer);
		DEBUG_PORT.flush();
	}
  }
