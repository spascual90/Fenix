/*
 * BearingMonitor.h
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#ifndef BEARINGMONITOR_H_
#define BEARINGMONITOR_H_

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

////#define SHIP_SIM // Uncomment to simulate boat to tune PID
//
//# define MAX_ITER 100000 // Max number of iterations to consider calibration procedure has failed
//# define CAL_CHECK_LOOP 7000//20000 // Number of iterations to check IMU Calibration results
//#define MAX_LOW_QDATA 100 // Maximum iterations with low quality data from IMU
//
////SDA - I2C data pin, connect to your microcontrollers I2C data line. This pin can be
////used with 3V or 5V logic, and there's a 10K pullup on this pin.
//#define PIN_SDA 20
//
////SCL - I2C clock pin, connect to your microcontrollers I2C clock line. This pin can be
//// used with 3V or 5V logic, and there's a 10K pullup on this pin
//#define PIN_SCL 21


enum e_IMU_status {NOT_DETECTED, OPERATIONAL, SIMULATED, CAL_MODE, EXTERNAL_IMU};
enum e_IMU_cal_status {CAL_NOT_STARTED, CAL_START, CAL_INPROGRESS, CAL_RESULT_RECALIBRATED, CAL_RESULT_NOT_CALIBRATED};
enum e_IMU_check {CHECK_NOT_STARTED, CHECK_ONGOING, CHECK_FINISHED};
enum e_internalIMU_status {INT_NOT_DETECTED, INT_DETECTED};

class Bearing_Monitor {
public:
	Bearing_Monitor(float headingDev);
	virtual ~Bearing_Monitor();

	adafruit_bno055_offsets_t const getSensorOffsets() {
		adafruit_bno055_offsets_t Calib;
		_bno.getSensorOffsets(Calib);
		return Calib;
	}


	void setIniCalib (adafruit_bno055_offsets_t iniCalib) {
		_iniCalib = iniCalib;
		setSensorOffsets();

	}


	int8_t const getTemp() {
		/* returns the current temperature */
		return _bno.getTemp();
	}

	int32_t getSensorId() {
		  sensor_t sensor;
		  _bno.getSensor(&sensor);
		  return sensor.sensor_id;
	}

	bool IMU_startCalibration(bool completeCal);
	bool compute_Cal_IMU(bool completeCal);
	bool compute_check_IMU(void);

	void displaySensorOffsets(void);
	void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);
	void displayCalStatus(void);
	void refreshCalStatus(void);

	bool getCalibrationStatus(uint8_t &system);
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
		reduce360(headingDev);
		_headingDev = headingDev;
	}

	int get_PIN_SDA() {return PIN_SDA;}
	int get_PIN_SCL() {return PIN_SCL;}

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

    float reduce360 (float value) {
    	if (value<0) value+= 360;
    	return fmod (value, 360.0);
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

protected:
    e_IMU_status setup(void);
    e_IMU_status updateHeading(bool fixedSource, bool valid, float HDM);
    bool IMU_startCalCheck(int maxLoop);
    void reset_calibration (void);
	//FUNCTIONAL MODULE:SHIP SIMULATOR
	void SIM_updateShip(int tillerAngle);


private:
	Adafruit_BNO055 _bno;
	adafruit_bno055_offsets_t _iniCalib; // Calibration offsets after recalibration process is OK
	void IBIT();
	float _roll=0;
	float _pitch=0;
	float _heading=0;
	// Permanent deviation
	float _headingDev=0;
	float _dm = 0; // Magnetic variation

	bool _heading_isValid = false;
	bool _heading_isFrozen = true;


	//Calibration settings
	e_IMU_status _IMU_status= NOT_DETECTED;
	e_internalIMU_status _internalIMU_status = INT_NOT_DETECTED;
	e_IMU_cal_status _IMU_cal_status= CAL_NOT_STARTED;
	e_IMU_check _IMU_check= CHECK_NOT_STARTED;
	int _cal_iter = 0;
	bool IMU_Cal_Loop(bool completeCal);
	bool IMU_CalCheck_Loop(void);
	uint16_t _x;
	int8_t _y;
	uint8_t _z;

	uint8_t _calSys, _calGyro, _calAccel, _calMagn;


	void setSensorOffsets(void) {
		setSensorOffsets(_iniCalib);
	}

	void setSensorOffsets(adafruit_bno055_offsets_t Calib) {
		_bno.setExtCrystalUse(false);// TEST
		_bno.setSensorOffsets(Calib);
		_bno.setExtCrystalUse(true);
	}

    e_IMU_status updateHeading();

    // EXTERNAL COMPASS
    e_IMU_status updateHeading(bool valid, float HDM);

	//FUNCTIONAL MODULE:SHIP SIMULATOR
	float _SIMheading =0;
	int unstat0= 10;//200;//	ddeg
	float alfaMax = 0.012;//	deg/mseg
	unsigned long deltaT=100;//	mseg
	float softFactor =10000;
	float intertia = 0.5;
};


#endif /* BEARINGMONITOR_H_ */
