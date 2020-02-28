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

# define MAX_ITER 1000 // Max number of iterations to consider calibration procedure has failed

//SDA - I2C data pin, connect to your microcontrollers I2C data line. This pin can be
//used with 3V or 5V logic, and there's a 10K pullup on this pin.
#define PIN_SDA 20

//SCL - I2C clock pin, connect to your microcontrollers I2C clock line. This pin can be
// used with 3V or 5V logic, and there's a 10K pullup on this pin
#define PIN_SCL 21


enum e_IMU_status {DETECTED, NOT_DETECTED, RECALIBRATED, NOT_CALIBRATED};

class Bearing_Monitor {
public:
	Bearing_Monitor(float headingDev);
	virtual ~Bearing_Monitor();

	adafruit_bno055_offsets_t const getSensorOffsets() {
		adafruit_bno055_offsets_t Calib;
		_bno.getSensorOffsets(Calib);
		return Calib;
	}

	void setSensorOffsets(adafruit_bno055_offsets_t Calib) {
		_bno.setSensorOffsets(Calib);
		_bno.setExtCrystalUse(true);
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


	e_IMU_status BNO_GetCal(bool wCheck = false);

	void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);
	void displayCalStatus(void);
	void displayIMULow(void);

	bool getCalibrationStatus(uint8_t &system);
	bool getCalibrationStatus(void);

	float getCurrentHeading() {
		return reduce360(_heading + _headingDev);
	}

	float getHeadingDev() const {
		return _headingDev;
	}

	void setHeadingDev(float headingDev = 0) {
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


protected:
    e_IMU_status setup(void);
    bool updateHeading();

private:
	Adafruit_BNO055 _bno;
	void IBIT();
	float _roll=0;
	float _pitch=0;
	float _heading=0;
	// Permanent deviation
	float _headingDev=0;
	float _dm = 0; // Magnetic variation

};


#endif /* BEARINGMONITOR_H_ */
