/*
 * BearingMonitorIMU.h
 *
 *  Created on: 12 Oct. 2022
 *      Author: Sergio
 */

#ifndef BEARINGMONITORIMU_H_
#define BEARINGMONITORIMU_H_

#include "BearingMonitorArq.h"

//#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
//
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"

#ifdef BNO055_INTERNAL_FUSION
	#include "RTIMUBNO055.h"
#else
	#include "RTIMU_ext.h"
#endif

#include "RTFusionRTQF.h"
#include "CalLib.h"

//TODO uncomment serial in RTMatch.cpp AND RTIMUMPU9150.cpp


// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

class Bearing_MonitorIMU : public Bearing_MonitorArq {
public:

	Bearing_MonitorIMU(float headingDev);
	~Bearing_MonitorIMU();

//	int8_t const getTemp() {
//		/* returns the current temperature */
//		return _bno.getTemp();
//	}

	int32_t getSensorId();

	//Offset management
	void displaySensorOffsets(void);

	//Calibration Management
	void refreshCalStatus(void);
	bool getCalibrationStatus(uint8_t &system);
//	void setIniCalib (adafruit_bno055_offsets_t &iniCalib) {
//		_iniCalib = iniCalib;
//		resetSensorOffsets();
//	}

protected:
    void IMU_setup_specific(long EE_address);
    bool IMU_startCalibration_specific (bool completeCal);
    bool IMU_startCalCheck(int maxLoop);
    void resetSensorOffsets(void) {
//		setSensorOffsets(_iniCalib);
	}


	unsigned char * getCalibrationString(void);
//	unsigned char * getCalibrationString( const adafruit_bno055_offsets_t &calibData);
	e_IMU_cal_status EEload_Calib_specific(long int &eeaddress);
	bool EEsave_Calib_specific( long int &eeaddress);

private:
	//Adafruit_BNO055 _bno;
#ifdef BNO055_INTERNAL_FUSION
	RTIMUBNO055 *_imu;
#else
	RTIMU_ext *_imu;                                           // the IMU object
#endif
	RTFusionRTQF _fusion;                                  // the fusion object
	RTIMUSettings _settings;                               // the settings object
	CALLIB_DATA _calData;                                  // the calibration data
//	adafruit_bno055_offsets_t _iniCalib; // Calibration offsets after recalibration process is OK
	void IBIT();
	bool IMU_Cal_Loop(bool completeCal);
	bool IMU_CalCheck_Loop(void);

	//void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);

//	void setSensorOffsets(const adafruit_bno055_offsets_t &Calib) {
//		_bno.setExtCrystalUse(false);// TEST
//		_bno.setSensorOffsets(Calib);
//		_bno.setExtCrystalUse(true);
//}

    e_IMU_status updateHeading();

};

#endif /* BEARINGMONITOR_H_ */
