/*
 * BearingMonitor.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "BearingMonitor.h"

#include "GPSport.h" // Include this library to output to DEBUG_PORT

Bearing_Monitor::Bearing_Monitor(float headingDev = 0) {
	setHeadingDev( headingDev);
}

Bearing_Monitor::~Bearing_Monitor() {
	// TODO Auto-generated destructor stub
}

e_IMU_status Bearing_Monitor::setup(void){
	_bno = Adafruit_BNO055(55);
	if(!_bno.begin()) return NOT_DETECTED;
	IBIT();
	return DETECTED;
}


void Bearing_Monitor::IBIT(){

	DEBUG_print("IMU int... Started\n");
	DEBUG_print("HW: BNO055\n");
	sprintf(DEBUG_buffer,"SDA,SCL=%i,%i\n",get_PIN_SDA(),get_PIN_SCL());
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
	//TODO: include routine to whait until bearing data is available (except calib IMU mode) While Sys<3 loop if not in x secs-->ask for calibration.
}

// check=true performs complete initial calibration + check (system==3 required)
// check=false ensures minimum recalibration after each power-on ( as long as mag and gyro are 3, data is realiable)
e_IMU_status Bearing_Monitor::BNO_GetCal(bool check){

		    int i =0;
		    bool calibrated=false;
		 	 sensors_event_t event;
		    _bno.getEvent(&event);

		    (check?DEBUG_print("Please Calibrate Sensor:\n"):DEBUG_print("Recalibrating Sensor:\n"));

		    // Loops until fully calibrated or calibration time exceeded
	        while ( !calibrated and (i++ < MAX_ITER))
	        {
	        	calibrated = (check==true?_bno.isFullyCalibrated():getCalibrationStatus());
	            _bno.getEvent(&event);

	            /* Display calibration status */
	            displayCalStatus();

	            /* Wait the specified delay before requesting new data */
	            delay(100);
	        }

	 	// If System status is lower than 3, IMU does not provide data of enough quality for calibration
		if (!calibrated) {
			DEBUG_print("\nCalibration time out!\n");
			DEBUG_print("WARNING: Calibration failed. Bearing values might be inaccurate.\n");
			return NOT_CALIBRATED; // Calibration time exceeded. Calibration failed
		}

        /* Display calibration status */
       displayCalStatus();

	    adafruit_bno055_offsets_t newCalib;
	    _bno.getSensorOffsets(newCalib);

	    DEBUG_print("\nCalibrated! Ok\n");
	    // Heading value is not received until a slight movement is detected by IMU
	    // Practicaly speaking this is not an issue, but some info is provided to user
	    updateHeading();
	    if (int(_heading)==0) DEBUG_print("Move slightly to start receiving IMU data\n");

	    if (check) {
	    	DEBUG_print("Calibration Results:\n");
		    displaySensorOffsets(newCalib);
			DEBUG_print("\nCheck Sensor Orientation:\n");
			i=0;

			while (i++ < MAX_ITER/10)
			{
				_bno.getEvent(&event);

				DEBUG_PORT.print("X: ");
				DEBUG_PORT.print(event.orientation.x, 0);
				DEBUG_PORT.print("\tY: ");
				DEBUG_PORT.print(event.orientation.y, 0);
				DEBUG_PORT.print("\tZ: ");
				DEBUG_PORT.print(event.orientation.z, 0);
				DEBUG_PORT.print("\n");
				/* Wait the specified delay before requesting new data */
				delay(500);
			}
	    }

	    return RECALIBRATED;

}  // End BNO055_Get_Cal

void Bearing_Monitor::displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    _bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    DEBUG_print("\t");
    if (!system)
    {
      DEBUG_print("! ");
    }

    /* Display the individual values */
	sprintf(DEBUG_buffer,"Sys:%i G:%i A:%i M:%i\n", system, gyro, accel, mag);
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
}

// updates heading even if data is not of sufficient quality
// return = true: accurate data; false= low data quality
bool Bearing_Monitor::updateHeading(){
	bool calStatus = getCalibrationStatus();
	// - VECTOR_EULER         - degrees
	imu::Vector<3> euler = _bno.getVector(Adafruit_BNO055::VECTOR_EULER);
	_heading = euler.x();
	return calStatus;
}

// fn available if system status value is not required
bool Bearing_Monitor::getCalibrationStatus(void) {
	uint8_t system;
	return getCalibrationStatus(system);
}

// return true if all sensors except accel are completely calibrated
// return false on the contrary
// Based on this solution to Technical Query, as long as mag and gyro are 3, data is realiable.
// https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BNO055-Calibration-Staus-not-stable/td-p/8375
bool Bearing_Monitor::getCalibrationStatus(uint8_t &system) {
	uint8_t gyro, accel, mag = 0;
	_bno.getCalibration(&system, &gyro, &accel, &mag);
	return (gyro==3 and mag==3);// (system==3 and gyro==3 and mag==3);
}

void Bearing_Monitor::displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{

	sprintf(DEBUG_buffer,"Accelerometer: x:%i, y:%i, z:%i\n",calibData.accel_offset_x, calibData.accel_offset_y, calibData.accel_offset_z);
	DEBUG_print(DEBUG_buffer);
	sprintf(DEBUG_buffer,"Gyro: x:%i, y:%i, z:%i\n",calibData.gyro_offset_x, calibData.gyro_offset_y, calibData.gyro_offset_z);
	DEBUG_print(DEBUG_buffer);
	sprintf(DEBUG_buffer,"Mag: x:%i, y:%i, z:%i\n",calibData.mag_offset_x, calibData.mag_offset_y, calibData.mag_offset_z);
	DEBUG_print(DEBUG_buffer);
	sprintf(DEBUG_buffer,"Accel Radius: %i\n",calibData.accel_radius);
	DEBUG_print(DEBUG_buffer);
	sprintf(DEBUG_buffer,"Mag Radius: %i\n",calibData.mag_radius);
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
}

