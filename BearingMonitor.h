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

constexpr float HEADING_ALFA = 0.99; //Higher: more stable but slower

enum e_IMU_status {NOT_DETECTED, OPERATIONAL, SIMULATED, CAL_MODE, EXTERNAL_IMU};
enum e_IMU_cal_status {CAL_NOT_STARTED, CAL_START, CAL_INPROGRESS, CAL_RESULT_RECALIBRATED, CAL_RESULT_NOT_CALIBRATED};
enum e_IMU_check {CHECK_NOT_STARTED, CHECK_ONGOING, CHECK_FINISHED};
enum e_internalIMU_status {INT_NOT_DETECTED, INT_DETECTED};

class BearingMonitor {
public:
	//BearingMonitor(float headingDev);
	BearingMonitor(float magneticVariation, float headingDev);
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


	bool getCalibrationStatus(void) {
		refreshCalStatus();
		return (_calSys==3);
	}

	bool getCheckXYZ(uint16_t &x, int8_t &y, uint8_t &z);
	bool getCheckSGAM(uint8_t &S, uint8_t &G, uint8_t &A, uint8_t &M);

	inline float getCurrentHeadingT(void) const {
		return _headingT;
	}

	inline float getCurrentHeadingC(void) const {
		return _heading;
	}

	inline float getHeadingDev() const {
		return (_IMU_status!= EXTERNAL_IMU?_headingDev:_ext_headingDev);
	}

	inline float getMagneticVariation() const {
		return (_IMU_status!= EXTERNAL_IMU?_magneticVariation:_ext_magneticVariation);
	}

	enum e_ref {TRUEtoMAGNETIC = 1, MAGNETICtoTRUE =-1};

    float ChangeRef(float value, e_ref ref){
    //M = T + dm; TRUEtoMAGNETIC
    //T = M - dm; MAGNETICtoTRUE

    	value+=ref*getMagneticVariation();
    	return reduce360(value);
    }

    inline float getTotalDev(void) const {
		return _totalDev;
	}

    inline bool isHeadingValid() const {
		return _heading_isValid;
	}

    inline bool isHeadingFrozen() const {
		return _heading_isFrozen;
	}

    inline e_IMU_status getIMUstatus() const {
		return _IMU_status;
	}

    inline e_IMU_check getIMUcheck() const {
		return _IMU_check;
	}

    inline e_IMU_cal_status getImuCalStatus() const {
		return _IMU_cal_status;
	}

    bool setIMUstatus (e_IMU_status newStatus ){
    	bool ret;
    	switch (newStatus) {
			case OPERATIONAL:
				if (_internalIMU_status == INT_DETECTED) {
					_IMU_status = OPERATIONAL;
					ret = true;
				} else {
					_IMU_status = NOT_DETECTED;
					ret = false;
				}
				break;
			default:
				_IMU_status = newStatus;
				ret = true;
				break;
    	}
    return ret;
    }

    void reset_calibration (long &eeaddress);

    void displaySensorOffsets(void) {
    	_imuDevice->displaySensorOffsets();
    }


	double deltaAngle(double origen, double destino) {
		double diff = fmod(destino - origen + 180.0, 360.0);
		if (diff < 0)
			diff += 360.0;
		return diff - 180.0;
	}



protected:
   e_IMU_status updateHeading(unsigned long HDT_RXtime);
	e_IMU_cal_status EEload_Calib(long int eeaddress);
	bool EEsave_Calib(long int eeaddress);
	bool set_calibrate_py_offsets(float B[3], float Ainv[3][3], char sensor);
	//FUNCTIONAL MODULE:SHIP SIMULATOR
	void SIM_updateShip(int tillerAngle);

	//float _ext_heading =0; // Raw magnetic heading value without mag.dev correction nor IMU installation heading correction
	// Permanent deviation
	float _ext_headingDev=0; // Magnetic Deviation: IMU installation/disturbances heading correction
	float _ext_magneticVariation=0; // Magnetic Variation: Depending on earth zone


	void setHeadingDev(float headingDev = 0) {
		_headingDev = reduce180(headingDev);
		updateTotalDev ();

	}

	void setMagneticVariation(float magneticVariation = 0) {
		_magneticVariation = reduce180(magneticVariation);
		updateTotalDev ();
	}

	void set_ext_heading_dt(float ext_heading_dt = 0) {
		_ext_heading_dt = ext_heading_dt;
		//DEBUG_sprintf("_ext_heading_dt",_ext_heading_dt);
	}

	void set_ext_headingDev(float headingDev = 0) {
		_ext_headingDev = headingDev;
		//DEBUG_sprintf("_ext_headingDev",_ext_headingDev);
		updateTotalDev ();
		//DEBUG_sprintf("_totalDev", _totalDev);
		//DEBUG_sprintf("_heading", _heading);
		//DEBUG_sprintf("_headingT", _headingT);

	}

	void set_ext_magneticVariation(float magneticVariation = 0) {
		_ext_magneticVariation = magneticVariation;
		//DEBUG_sprintf("_ext_magneticVariation",_ext_magneticVariation);
		updateTotalDev ();
	}

	float predictYawDelta(float dt);

private:
	IMUDevice *_imuDevice;

	float _roll=0;
	float _pitch=0;
	//variables independientes
	float _heading=0; // Raw magnetic heading value without mag.dev correction nor IMU installation heading correction
	// Permanent deviation
	float _ext_heading_dt=0; // Static value of heading received from HDG/HDT. Value will be extrapolated to _heading until new message is received
	float _headingDev=0; // Magnetic Deviation: IMU installation/disturbances heading correction
	float _magneticVariation=0; // Magnetic Variation: Depending on earth zone

	//variables derivadas
	float _totalDev = 0;
	float _headingT=0;

	void updateTotalDev(void) {
		_totalDev = (_IMU_status !=EXTERNAL_IMU? _headingDev + _magneticVariation:_ext_headingDev + _ext_magneticVariation);
		updateHeadingT();
	}

	void updateHeadingT(void) {
		_headingT = reduce360(_heading + getTotalDev());

	}

	bool _heading_isValid = false; // Availability of heading
	bool _heading_isFrozen = true; // Quality of heading value

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

	// INTERNAL COMPASS ONLY
	void updateHeadingINT(void);

    // EXTERNAL COMPASS ONLY
    e_IMU_status updateHeadingEXT(unsigned long HDT_RXtime, bool ab_reset);

	//FUNCTIONAL MODULE:SHIP SIMULATOR
	float _SIMheading =0;
	int unstat0= 0;//200;//	ddeg
	float alfaMax = 0.012;//	deg/mseg
	unsigned long deltaT=100;//	mseg
	float softFactor =10000;
	float intertia = 0.5;
};


#endif /* BEARINGMONITOR_H_ */
