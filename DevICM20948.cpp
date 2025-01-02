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

	DEBUG_print(F("IMU int... Started\n"));
	sprintf(DEBUG_buffer,"HW: %s\nSDA,SCL=%i,%i\n", IMUName(), get_PIN_SDA(),get_PIN_SCL());

	DEBUG_print(DEBUG_buffer);
	DEBUG_flush();
}

float DevICM20948::updateHeading(void){
	return ICM20948AHRS_loop();
}


bool DevICM20948::EEload_Calib(long int &eeAddress)
{
	//  Get and restore Calibration offsets
	long id_IMUEE, id_IMU;
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
	EEPROM.get(eeAddress, id_IMUEE);

	// Look for unique ID reported by IMU
	id_IMU = get_IMUdeviceID();
	if (id_IMUEE != id_IMU) {
		DEBUG_print(F("!W:No Calib. Data for sensor."));
		DEBUG_print(F(" Using default values!\n"));
		sprintf(DEBUG_buffer,"ID found: %i\n",id_IMUEE);
		DEBUG_print(DEBUG_buffer);

		//Gyro default scale 250 dps. Convert to radians/sec subtract offsets
//		float lG_offset[3] = {240.7, 227.7, -4.8};
//
//		//Accel scale: divide by 16604.0 to normalize
//
//		float lA_B[3] = { -121.55 , -7.98 , 26.18 };
//		float lA_Ainv[3][3] = {
//		{ 3.21418 , -0.0725 , 0.016 },
//		{ -0.0725 , 3.18079 , -0.01859 },
//		{ 0.016 , -0.01859 , 3.16865 }};
//
//		//Mag scale divide by 369.4 to normalize
//
//		float lM_B[3] = { -117.42 , 7.33 , 34.17 };
//
//		float lM_Ainv[3][3] = {
//		{ 3.18336 , -0.04678 , -0.0167 },
//		{ -0.04678 , 3.18373 , -0.03753 },
//		{ -0.0167 , -0.03753 , 3.13766 }};
		float hcG_offset[3] = {
				0,0,0 };
		//float hcG_offset[3] = { 288.12,249.14,-37.53 };
		float hcA_B[3] = {
				0,0,0 };
		//-85.72,-276.92,315.30 };


		float hcA_Ainv[3][3] = {
		{ 0.06068 , 0.00243 , -9e-05 },
		{ 0.00243 , 0.06124 , -2e-05 },
		{ -9e-05 , -2e-05 , 0.06027 }};

		float hcM_B[3] = {
				0,0,0 };
		//-119.43,8.69,33.04 };


		float hcM_Ainv[3][3] = {
		{ 4.16936 , -0.11603 , 0.02637 },
		{ -0.11603 , 4.08146 , 0.00833 },
		{ 0.02637 , 0.00833 , 4.07421 }};
		//ICM20948: mandar valores de variables calibración
		ICM20948AHRS_setOffsets(hcG_offset, hcA_B, hcA_Ainv,  hcM_B, hcM_Ainv);
		displaySensorOffsets();
		return false;

	} else {
		DEBUG_print(F("!Found Calibration data...\n"));
		_gyro=3;
		_accel=3;
		_mag=3;
	//ICM20948: Cargar valores de variables calibración
	eeAddress += sizeof(id_IMU);
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
	long id_IMU;
	// Variables calibración ICM20948
	float G_offset[3];
	float A_B[3];
	float A_Ainv[3][3];
	float M_B[3];
	float M_Ainv[3][3];

	DEBUG_print(F("!Saving Calibration..."));

	//ID
	id_IMU = get_IMUdeviceID();
	EEPROM.put(eeAddress, id_IMU);

	eeAddress += sizeof(id_IMU);
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

	DEBUG_print(F("Ok\n"));

    return DataStored;

}

void DevICM20948::displaySensorOffsets(void){
	//ICM20948: Recibir valores de variables calibración e imprimirlos
	//l = minima longitud incluyendo . y -
	//d = número de cifras decimales
	//e = máximo número de cifras enteras sin incluir -
	int d2=2, l2=3+d2;
	int d5=5, l5=3+d5 ;
	char c1[5 + l5];
	char c2[5 + l5];
	char c3[5 + l5];
	// Variables calibración ICM20948
	float G_offset[3];
	float A_B[3];
	float A_Ainv[3][3];
	float M_B[3];
	float M_Ainv[3][3];

	ICM20948AHRS_getOffsets(G_offset, A_B, A_Ainv, M_B,  M_Ainv);

	if (!ICM20948AHRS_checkOffsets(G_offset, A_B, A_Ainv, M_B,  M_Ainv)) {
		DEBUG_print(F("!IMU offsets out of range\n"));
		DEBUG_flush();
		return;
	}

	sprintf(DEBUG_buffer,"G_offset: x:%s, y:%s, z:%s\n",
			dtostrf(G_offset[0],l2,d2,c1),
			dtostrf(G_offset[1],l2,d2,c2),
			dtostrf(G_offset[2],l2,d2,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_flush();
	sprintf(DEBUG_buffer,"A_B: %s \t %s \t %s\n",
			dtostrf(A_B[0],l2,d2,c1),
			dtostrf(A_B[1],l2,d2,c2),
			dtostrf(A_B[2],l2,d2,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_flush();
	sprintf(DEBUG_buffer,"A_Ainv:\t %s \t %s \t %s\n",
			dtostrf(A_Ainv[0][0],l5,d5,c1),
			dtostrf(A_Ainv[0][1],l5,d5,c2),
			dtostrf(A_Ainv[0][2],l5,d5,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_flush();
	sprintf(DEBUG_buffer,"A_Ainv:\t %s \t %s \t %s\n",
			dtostrf(A_Ainv[1][0],l5,d5,c1),
			dtostrf(A_Ainv[1][1],l5,d5,c2),
			dtostrf(A_Ainv[1][2],l5,d5,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_flush();
	sprintf(DEBUG_buffer,"A_Ainv:\t %s \t %s \t %s\n",
			dtostrf(A_Ainv[2][0],l5,d5,c1),
			dtostrf(A_Ainv[2][1],l5,d5,c2),
			dtostrf(A_Ainv[2][2],l5,d5,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_flush();
	sprintf(DEBUG_buffer,"M_B: %s \t %s \t %s\n",
			dtostrf(M_B[0],l2,d2,c1),
			dtostrf(M_B[1],l2,d2,c2),
			dtostrf(M_B[2],l2,d2,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_flush();
	sprintf(DEBUG_buffer,"M_Ainv: %s \t %s \t %s\n",
			dtostrf(M_Ainv[0][0],l5,d5,c1),
			dtostrf(M_Ainv[0][1],l5,d5,c2),
			dtostrf(M_Ainv[0][2],l5,d5,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_flush();
	sprintf(DEBUG_buffer,"M_Ainv: %s \t %s \t %s\n",
			dtostrf(M_Ainv[1][0],l5,d5,c1),
			dtostrf(M_Ainv[1][1],l5,d5,c2),
			dtostrf(M_Ainv[1][2],l5,d5,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_flush();
	sprintf(DEBUG_buffer,"M_Ainv: %s \t %s \t %s\n",
			dtostrf(M_Ainv[2][0],l5,d5,c1),
			dtostrf(M_Ainv[2][1],l5,d5,c2),
			dtostrf(M_Ainv[2][2],l5,d5,c3));
	DEBUG_print(DEBUG_buffer);
	DEBUG_flush();
}

// CALIBRATION FUNCTIONS
// Launch all sensors: $PEMC,09,-*3F
bool DevICM20948::IMU_startCalibration(char sensor) {
	_gyro = 1;
	_accel = 1;
	_mag = 1;
	float gyro[3];

	sensor =='-'?_allsensor = true:_allsensor = false;
	_allsensor==true?_sensor = 'G':_sensor = sensor;

	return true;
}


// return false when calibration is finished
// true, calibration ongoing
bool DevICM20948::IMU_Cal_Loop(void){
	bool ret= true;
	static int print_time = 0;
	int16_t* raw_sensor;


	if (_sensor_count == 0) {
		_sensor_count =1;
		switch (_sensor) {
		case 'G':
			// read from sensor and write
			// First read of this sensor
			DEBUG_print(F("!Gyro raw data:\n"));
			DEBUG_print(F("Keep device stable...\n"));
			DEBUG_print(F("$PEMC,12,O,0,1,0,0*56\n"));
			break;
		case 'A':
			DEBUG_print(F("!Accel. raw data:\n"));
			DEBUG_print(F("6 stable positions...\n"));
			DEBUG_print(F("$PEMC,12,O,0,0,1,0*56\n"));
			break;
		case 'M':
			DEBUG_print(F("!Magnet. raw data:\n"));
			DEBUG_print(F("Write number 8...\n"));
			DEBUG_print(F("$PEMC,12,O,0,0,0,1*56\n"));
			break;
		}
	}

	// read from sensor and write
	raw_sensor = ICM20948AHRS_calibration_loop (_sensor);
	if (print_time++ == SENSOR_PRINTS) {
		displayRaw_sensorOffsets(raw_sensor);
		print_time = 0;
		_sensor_count++;
	}
	// evaluate continue sensor, next sensor or finish
	if (_sensor_count == SENSOR_LOOPS) {
		_sensor_count=0;
		DEBUG_print(F("!Raw data sent\n"));
		if (_allsensor == true) {
			switch (_sensor) {
			case 'G':
				_sensor = 'A'; // Next sensor
				ret = true;
				break;
			case 'A':
				_sensor = 'M'; // Next sensor
				ret = true;
				break;
			case 'M':
				ret = false; // Finished calibration of all sensors
				break;
			}
		} else {
			ret = false; // Finished calibration of selected sensor
		}
	}

	if (ret == false) DEBUG_print(F("$PEMC,12,N,0,0,0,0*57\n")); // After offset sending, IMU is not calibrated

	return ret;
}



void DevICM20948::Cal_NextSensor(void) {
	// Fuerza a cambiar de sensor llegando al fin del conteo
	if (_sensor_count!= 0) _sensor_count = SENSOR_LOOPS -1;
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

//void DevICM20948::displayRaw_gyroOffsets(float *gyro) {
//	sprintf(DEBUG_buffer,"Gyro offsets: %i \t %i \t %i\n", gyro[0],gyro[1],gyro[2]);
//	DEBUG_print(DEBUG_buffer);
//	DEBUG_flush();
//  }

void DevICM20948::displayRaw_sensorOffsets(int16_t *raw_sensor) {
	sprintf(DEBUG_buffer,"%i,%i,%i\n",raw_sensor[0],raw_sensor[1],raw_sensor[2]);
	DEBUG_print(DEBUG_buffer);
	DEBUG_flush();
  }

//void DevICM20948::displayRaw_SumOffsets(int16_t **acc_mag) {
//	int i=0;
//	DEBUG_print(F("Accelerometer raw data:\n"));
//	for (i=0; i<SENSOR_LOOPS; i++){
//		sprintf(DEBUG_buffer,"%i,%i,%i\n",acc_mag[i][0],acc_mag[i][1],acc_mag[i][2]);
//		DEBUG_print(DEBUG_buffer);
//		DEBUG_flush();
//	}
//
//	DEBUG_print(F("Magnet raw data:\n"));
//	for (i=0; i<SENSOR_LOOPS; i++){
//		sprintf(DEBUG_buffer,"%i,%i,%i\n",acc_mag[i][3],acc_mag[i][4],acc_mag[i][5]);
//		DEBUG_print(DEBUG_buffer);
//		DEBUG_flush();
//	}
//  }

bool DevICM20948::set_calibrate_py_offsets(float B[3], float Ainv[3][3], char sensor) {
	ICM20948AHRS_setoffsets2(B, Ainv, sensor);
	displaySensorOffsets();

	return true;
}


