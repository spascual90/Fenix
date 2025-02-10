/*
 * BearingMonitor.h
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#ifndef BEARINGMONITOR_H_
#define BEARINGMONITOR_H_

#include <utility/imumaths.h>
#include <EEPROM.h>

// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"
#include "GPSport.h" // Ports configuration
#include "IMUDevice.h"

enum e_IMU_status {NOT_DETECTED, OPERATIONAL, SIMULATED, CAL_MODE, EXTERNAL_IMU};
enum e_IMU_cal_status {CAL_NOT_STARTED, CAL_START, CAL_INPROGRESS, CAL_RESULT_RECALIBRATED, CAL_RESULT_NOT_CALIBRATED};
enum e_IMU_check {CHECK_NOT_STARTED, CHECK_ONGOING, CHECK_FINISHED};
enum e_internalIMU_status {INT_NOT_DETECTED, INT_DETECTED};

class BearingMonitor {
public:
	BearingMonitor(float headingDev);
	virtual ~BearingMonitor();
	e_IMU_status IMU_setup(long EE_address);
	void IBIT();

	bool IMU_startCalibration(char sensor);
	bool compute_Cal_IMU(char sensor);
	void Cal_NextSensor(void);
	bool isExternalCalibration(void);

	bool compute_check_IMU(void);

	void displayCalStatus(void);
	void refreshCalStatus(void);

	bool getCalibrationStatus(void);

	bool getCheckXYZ(uint16_t &x, int8_t &y, uint8_t &z);
	bool getCheckSGAM(uint8_t &S, uint8_t &G, uint8_t &A, uint8_t &M);

	float getCurrentHeading() {
		//In EXTERNAL_IMU mode, Heading deviation is not managed by Autopilot but by external device
		return (_IMU_status==EXTERNAL_IMU?reduce360(_heading):reduce360(_heading + _headingDev));

	}

	float getHeadingDev() const {
		return _headingDev;
	}

	void setHeadingDev(float headingDev = 0) {
		_headingDev = reduce360(headingDev);
	}

	float getDm() const {
		return _dm;
	}
	void setDm(float dm = 0) {
		_dm = dm;
	}

    enum e_ref {TRUEtoMAGNETIC = 1, MAGNETICtoTRUE =-1};

    float ChangeRef(float value, e_ref ref){
    //M = T + dm; TRUEtoMAGNETIC
    //T = M - dm; MAGNETICtoTRUE

    	value+=ref*getDm();
    	return reduce360(value);
    }


	bool isHeadingValid() const {
		return _heading_isValid;
	}

	bool isHeadingFrozen() const {
		return _heading_isFrozen;
	}

	e_IMU_status getIMUstatus() const {
		return _IMU_status;
	}

	e_IMU_check getIMUcheck() const {
		return _IMU_check;
	}

	e_IMU_cal_status getImuCalStatus() const {
		return _IMU_cal_status;
	}

    void reset_calibration (long &eeaddress);

    void displaySensorOffsets(void) {
    	_imuDevice->displaySensorOffsets();
    }

protected:
   e_IMU_status updateHeading(bool fixedSource, bool valid, float HDM);
	e_IMU_cal_status EEload_Calib(long int eeaddress);
	bool EEsave_Calib(long int eeaddress);
	bool set_calibrate_py_offsets(float B[3], float Ainv[3][3], char sensor);
	//FUNCTIONAL MODULE:SHIP SIMULATOR
	void SIM_updateShip(int tillerAngle);

private:
	IMUDevice *_imuDevice;

	float _roll=0;
	float _pitch=0;
	float _heading=0;
	// Permanent deviation
	float _headingDev=0;
	float _dm = 0; // Magnetic variation

	bool _heading_isValid = false;
	bool _heading_isFrozen = true;
	float _heading_alfa = 0.99; //Higher: more stable but slower


	//Calibration settings
	e_IMU_status _IMU_status= NOT_DETECTED;
	e_internalIMU_status _internalIMU_status = INT_NOT_DETECTED;
	e_IMU_cal_status _IMU_cal_status= CAL_NOT_STARTED;
	e_IMU_check _IMU_check= CHECK_NOT_STARTED;
	//int _cal_iter = 0;

	uint16_t _x;
	int8_t _y;
	uint8_t _z;

	uint8_t _calSys, _calGyro, _calAccel, _calMagn;

	bool IMU_startCalCheck(void);
	bool IMU_Cal_Loop(void);
	bool IMU_CalCheck_Loop(void);

	//virtual
	void resetSensorOffsets(void);

	void updateHeading(void);

    // EXTERNAL COMPASS
    e_IMU_status updateHeading(bool valid, float HDM);

	//FUNCTIONAL MODULE:SHIP SIMULATOR
	float _SIMheading =0;
	int unstat0= 0;//200;//	ddeg
	float alfaMax = 0.012;//	deg/mseg
	unsigned long deltaT=100;//	mseg
	float softFactor =10000;
	float intertia = 0.5;
};


#endif /* BEARINGMONITOR_H_ */
