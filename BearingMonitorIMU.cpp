/*
 * BearingMonitor.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "BearingMonitorIMU.h"

Bearing_MonitorIMU::Bearing_MonitorIMU(float headingDev)
:Bearing_MonitorArq(headingDev) {
}

Bearing_MonitorIMU::~Bearing_MonitorIMU() {
	// TODO Auto-generated destructor stub
}

void Bearing_MonitorIMU::IMU_setup_specific(long EE_address){
//	_bno = Adafruit_BNO055(55);
//	if(_bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
//		_internalIMU_status= INT_DETECTED;
//		_IMU_status = CAL_MODE;
//		_IMU_cal_status = CAL_START;
//		IBIT();
//	} else {
//		_IMU_status = NOT_DETECTED;
//		_internalIMU_status= INT_NOT_DETECTED;
//	}

//#if !defined(BNO055_28)
//	#error "!Compilation error BNO055_28 must be selected in RTIMULibdefs.h"
//#endif

    Wire.begin();

#ifdef BNO055_INTERNAL_FUSION
    _imu = (RTIMUBNO055 *)RTIMU::createIMU(&_settings);                        // create the imu object
#else
    _imu = RTIMU_ext::createIMU(&_settings);
    _imu->setEeAddress(EE_address);
#endif
	   int errcode;
	    if ((errcode = _imu->IMUInit()) < 0) {
	    	sprintf(DEBUG_buffer,"IMU %s. Error Code: %i\n",_imu->IMUName(), errcode);
	    	DEBUG_print(DEBUG_buffer);
	    	DEBUG_PORT.flush();

			_IMU_status = NOT_DETECTED;
			_internalIMU_status= INT_NOT_DETECTED;
			return;
	    }

		_internalIMU_status= INT_DETECTED;
		_IMU_status = CAL_MODE;
		_IMU_cal_status = CAL_START;

	    // Slerp power controls the fusion and can be between 0 and 1
	    // 0 means that only gyros are used, 1 means that only accels/compass are used
	    // In-between gives the fusion mix.

	    _fusion.setSlerpPower(0.02);

	    // use of sensors in the fusion algorithm can be controlled here
	    // change any of these to false to disable that sensor

	    _fusion.setGyroEnable(true);
	    _fusion.setAccelEnable(true);
	    _fusion.setCompassEnable(true);

		IBIT();

}

void Bearing_MonitorIMU::IBIT(){

	DEBUG_print("IMU int... Started\n");
	sprintf(DEBUG_buffer,"HW: %s\nSDA,SCL=%i,%i\n",_imu->IMUName(), get_PIN_SDA(),get_PIN_SCL());

	DEBUG_print("Sensor fusion: ");
	#ifdef BNO055_INTERNAL_FUSION
	DEBUG_print("BNO055 Internal Fusion Mode\n");
	#else
	DEBUG_print("All IMU External Fusion Mode\n");
	#endif


	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
}

int32_t Bearing_MonitorIMU::getSensorId() {
	  //sensor_t sensor;
	  //_bno.getSensor(&sensor);
	  //return sensor.sensor_id;
	return _imu->IMUType();
}

bool Bearing_MonitorIMU::IMU_startCalibration_specific(bool completeCal) {

	//This driver does not calibrate dynamically
	if (completeCal == false) return false;

	//First iteration only
	_imu->setCalibrationMode(true);                     // make sure we get raw data
	sprintf(DEBUG_buffer,"ArduinoIMU calibrating device %s\n",_imu->IMUName());
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();

	return true;
}

bool Bearing_MonitorIMU::IMU_startCalCheck(int max_loop) {
	_cal_iter = max_loop;
	_IMU_check = CHECK_ONGOING;
	return true;
}

//return true-->process ongoing
//return false-->process finished
bool Bearing_MonitorIMU::IMU_Cal_Loop(bool completeCal){

	bool calibrated;

	// Exit if calibration is not in progress
	if (_IMU_cal_status != CAL_INPROGRESS) return false;

	//This driver does not calibrate dynamically
	if (completeCal == false) return false;

	  boolean changed;
	  RTVector3 mag;

	  if (_imu->IMURead()) {                                 // get the latest data
	    changed = false;
	    mag = _imu->getCompass();
	    for (int i = 0; i < 3; i++) {
	      if (mag.data(i) < _calData.magMin[i]) {
	        _calData.magMin[i] = mag.data(i);
	        changed = true;
	      }
	      if (mag.data(i) > _calData.magMax[i]) {
	        _calData.magMax[i] = mag.data(i);
	        changed = true;
	      }
	    }

	    if (changed) _cal_iter=0; //Reset iteration if new data is found

	  }

	// Loops until calibration time exceeded
	if (_cal_iter++ == MAX_ITER) {
		// Calibration period exceeded
		_IMU_cal_status==CAL_RESULT_RECALIBRATED;
		refreshCalStatus();
		return false;
	}

	return true;
}

bool Bearing_MonitorIMU::IMU_CalCheck_Loop(void){
	_IMU_check = CHECK_FINISHED;

	return false;
}

void Bearing_MonitorIMU::refreshCalStatus(void)
{
//    /* Get the four calibration values (0..3) */
//    /* Any sensor data reporting 0 should be ignored, */
//    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
//    _imu->getCalibration(&system, &gyro, &accel, &mag);

//    _calSys = system;
//    _calGyro = gyro;
//    _calAccel = accel;
//    _calMagn = mag;
	_calSys = 3;
	_calGyro = 3;
	_calAccel = 3;
	_calMagn = 3;
}

// updates heading even if data is not of sufficient quality
// return = true: accurate data; false= low data quality
e_IMU_status Bearing_MonitorIMU::updateHeading(){
//	bool calStatus = true;
//	bool valid_data = false;
//	static int low_quality_data=0;
//
//	if (_IMU_status == EXTERNAL_IMU) return _IMU_status;
//
//	// heading is valid only when a good reference is obtained from calibration status
//	// heading is invalid at the begining of the execution and after 2 loops of bad quality data
//	if (Bearing_MonitorArq::getCalibrationStatus()) {
//		_heading_isValid = true;
//		_heading_isFrozen = false;
//		 valid_data = true;
//	} else {
//		 valid_data = false;
//
//	}
//
//	// - VECTOR_EULER         - degrees
//	imu::Vector<3> euler = _bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//	float read_heading = euler.x();
//
//	//Only updates if heading is not frozen
//	if ((_heading_isValid==true) and (_heading_isFrozen==false)) {
//		_heading = read_heading;
//
//
//	}
//
//	if (!valid_data) {
//		low_quality_data++;
//
//		if (low_quality_data>MAX_LOW_QDATA) {
//			if (_heading_isFrozen==false) {
//				// Low quality data and last data was not frozen. This is the first time to get low quality data for a while after last calibration reset.
//				// Freeze Heading (last value still valid) until valid data is received from IMU.
//				//IMU calibration reset in ALL operational modes (not only STAND_BY)
//				reset_calibration(); // Reset calibration in all operational modes. Except when external IMU is in use.
//				low_quality_data=0;
//				_heading_isFrozen = true;
//			} else {
//				//Low quality data for two times in a row. Heading is not valid any more
//				_heading_isValid = false;
//			}
//		}
//	}
//	return _IMU_status;

	if (_IMU_status == EXTERNAL_IMU) return _IMU_status;
	_heading_isFrozen=false;
	_heading_isValid=true;

   	int loopCount = 1;

    while (_imu->IMURead()) {
    	if (++loopCount >= 10)  // this flushes remaining data in case we are falling behind
    		continue;

        _fusion.newIMUData(_imu->getGyro(), _imu->getAccel(), _imu->getCompass(), _imu->getTimestamp());
        _heading = reduce360(((RTVector3&)_fusion.getFusionPose()).z() * RTMATH_RAD_TO_DEGREE);

    }

	//_IMU_status == OPERATIONAL;

	return _IMU_status;
}

// return true if sensors are enough calibrated
// return false on the contrary
// Based on this solution to Technical Query, as long as mag and gyro are 3, data is realiable.
// https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BNO055-Calibration-Staus-not-stable/td-p/8375
// Due to difficulties in calibration, required sensor calibration status has been reducted.

bool Bearing_MonitorIMU::getCalibrationStatus(uint8_t &system) {
	uint8_t gyro, accel, mag = 0;
	//_imu->getCalibration(&system, &gyro, &accel, &mag);
	return (gyro>1 and mag>1);// (gyro==3 and mag==3); (system==3 and gyro==3 and mag==3);
	return true;
}

void Bearing_MonitorIMU::displaySensorOffsets()
{
//	adafruit_bno055_offsets_t newCalib;
//	if (_bno.getSensorOffsets(newCalib)) {
//		DEBUG_print("displaySensorOffsets: Fully calibrated\n");
//		displaySensorOffsets(newCalib);
//	} else {
//		DEBUG_print("displaySensorOffsets: Not fully calibrated\n");
//	}
//	DEBUG_print("displaySensorOffsets: Not fully calibrated\n");
//
DEBUG_print("displaySensorOffsets\n");
}

//void Bearing_MonitorIMU::displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
//{
//
//	sprintf(DEBUG_buffer,"Accelerometer: x:%i, y:%i, z:%i\n",calibData.accel_offset_x, calibData.accel_offset_y, calibData.accel_offset_z);
//	DEBUG_print(DEBUG_buffer);
//	sprintf(DEBUG_buffer,"Gyro: x:%i, y:%i, z:%i\n",calibData.gyro_offset_x, calibData.gyro_offset_y, calibData.gyro_offset_z);
//	DEBUG_print(DEBUG_buffer);
//	sprintf(DEBUG_buffer,"Mag: x:%i, y:%i, z:%i\n",calibData.mag_offset_x, calibData.mag_offset_y, calibData.mag_offset_z);
//	DEBUG_print(DEBUG_buffer);
//	sprintf(DEBUG_buffer,"Accel Radius: %i\n",calibData.accel_radius);
//	DEBUG_print(DEBUG_buffer);
//	sprintf(DEBUG_buffer,"Mag Radius: %i\n",calibData.mag_radius);
//	DEBUG_print(DEBUG_buffer);
//	DEBUG_PORT.flush();
//
//
//}

e_IMU_cal_status Bearing_MonitorIMU::EEload_Calib_specific(long int & eeAddress)
{

    if (_imu->getCalibrationValid())
    	DEBUG_print("Using compass calibration\n");
    else
    	DEBUG_print("Compass calibration data not required\n");

//	//  Get and restore BNO Calibration offsets
//	long EE_bnoID, bnoID;
//	adafruit_bno055_offsets_t calibrationData;
//	//  Look for the sensor's unique ID in EEPROM.
//	EEPROM.get(eeAddress, EE_bnoID);
//	// Look for unique ID reported by IMU
//	bnoID = Bearing_MonitorIMU::getSensorId();
//	sprintf(DEBUG_buffer,"IMU Sensor ID: %i\n",bnoID);
//	DEBUG_print(DEBUG_buffer);
//
//	if (EE_bnoID != bnoID) {
//		DEBUG_print("!WARNING: No Calibration Data for this sensor found!\n");
//		sprintf(DEBUG_buffer,"ID found: %i\n",EE_bnoID);
//		DEBUG_print(DEBUG_buffer);
//
//		return CAL_RESULT_NOT_CALIBRATED;
//	}
//	DEBUG_print("!Found Calibration data...\n");
//	eeAddress += sizeof(long);
//	EEPROM.get(eeAddress, calibrationData);
//	displaySensorOffsets(calibrationData);
//	setIniCalib(calibrationData);
//	displaySensorOffsets();

	return CAL_RESULT_RECALIBRATED;
}

bool Bearing_MonitorIMU::EEsave_Calib_specific( long &eeAddress){
	bool DataStored = false;
// 	//DATA TO SAVE
//	long bnoID;
//	adafruit_bno055_offsets_t Calib;
//
//	DEBUG_print("!Saving Calibration...");
//
//	//bnoID
//    bnoID = getSensorId();
//    EEPROM.put(eeAddress, bnoID);
//    eeAddress += sizeof(bnoID);
//
//    //Calib
//	_bno.getSensorOffsets(Calib);
//    EEPROM.put(eeAddress, Calib);

    DataStored = true;

	DEBUG_print("Ok\n");
    return DataStored;
}  //  end EEsave_Calib


unsigned char * Bearing_MonitorIMU::getCalibrationString(void) {
//	adafruit_bno055_offsets_t Calib;
//	_bno.getSensorOffsets(Calib);
//	return getCalibrationString(Calib);
return "";
}

//unsigned char * Bearing_MonitorIMU::getCalibrationString( const adafruit_bno055_offsets_t &Calib) {
//	long bnoID;
//
//	//bnoID
//    bnoID = getSensorId();
//
//    int len_bnoID = sizeof(bnoID);
//    int len_Calib = sizeof(Calib);
//
//    unsigned char * raw = malloc(len_bnoID+len_Calib);
//
//    memcpy(raw, &bnoID, len_bnoID);
//    memcpy(raw + len_bnoID, &Calib, len_Calib);
//
//    memcpy(&bnoID, raw, len_bnoID);
//    raw += len_bnoID;
//    memcpy(&Calib, raw , len_Calib);

//    return "";
//}
