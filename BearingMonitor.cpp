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
	_IMU_status = NOT_DETECTED;
}

e_IMU_status Bearing_Monitor::setup(void){
	_bno = Adafruit_BNO055(55);
	if(_bno.begin(Adafruit_BNO055::OPERATION_MODE_NDOF)) {
		_IMU_status = CAL_MODE;
		_IMU_cal_status = CAL_START;
		IBIT();
	} else {
#ifdef SHIP_SIM
		_IMU_status = SIMULATED;
#else
		_IMU_status = NOT_DETECTED;
#endif
	}
	return _IMU_status;
}


void Bearing_Monitor::IBIT(){

	DEBUG_print("IMU int... Started\n");
	DEBUG_print("HW: BNO055\n");
	sprintf(DEBUG_buffer,"SDA,SCL=%i,%i\n",get_PIN_SDA(),get_PIN_SCL());
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
	//TODO: include routine to wait until bearing data is available (except calib IMU mode) While Sys<3 loop if not in x secs-->ask for calibration.
}

void Bearing_Monitor::reset_calibration () {
	DEBUG_print("***calib\n");
	setup();
	setSensorOffsets();
	displaySensorOffsets();
	_IMU_status = CAL_MODE;
	_IMU_cal_status = CAL_START;

}

// Recurrent loop for calibration
//CAL_START-->CAL_INPROGRESS-->(CAL_RESULT_NOT_CALIBRATED; CAL_RESULT_RECALIBRATED)
//CAL_RESULT_RECALIBRATED-->completeCal?-->CHECK_ONGOING-->CHECK_FINISHED
//return true-->process ongoing
//return false-->process finished / IMU in operational mode again
bool Bearing_Monitor::compute_Cal_IMU(bool completeCal) {
	bool ret=false;
	switch (_IMU_cal_status) {
	case CAL_NOT_STARTED:
		ret=false;
		break;
	case CAL_START:
		ret=Bearing_Monitor::IMU_startCalibration(completeCal);
		break;
	case CAL_INPROGRESS:
		ret=Bearing_Monitor::IMU_Cal_Loop(completeCal);
		break;
	case CAL_RESULT_NOT_CALIBRATED:
		ret=false;
		break;
	case CAL_RESULT_RECALIBRATED:
		// Check calibration status
		// completeCal=true performs complete initial calibration + check (system==3 required)
		// completeCal=false ensures minimum recalibration after each power-on ( as long as mag and gyro are 3, data is realiable)
		if (completeCal) {
			ret=compute_check_IMU();
		} else {
			ret=false;
		}
		break;
	default:
		ret=false;
		break;
	}
	if (ret == false and _IMU_status != OPERATIONAL) {
		_IMU_status = OPERATIONAL;
		DEBUG_print("***Finished calib\n");
	}
	return ret;
}

//return true-->process ongoing
//return false-->process finished
bool Bearing_Monitor::compute_check_IMU(void) {

	switch (_IMU_check) {
	case CHECK_NOT_STARTED:
		//start check
		return IMU_startCalCheck(CAL_CHECK_LOOP);
		break;
	case CHECK_ONGOING:
		return Bearing_Monitor::IMU_CalCheck_Loop();
		break;
	case CHECK_FINISHED:
		return false;
		break;
	}

	return (_IMU_check ==CHECK_ONGOING);
}


bool Bearing_Monitor::IMU_startCalibration(bool completeCal) {
	_cal_iter = 0;
	_IMU_cal_status = CAL_INPROGRESS;

	//First iteration only
	(completeCal?DEBUG_print("IMU Calibration: Complete + Check.\n"):DEBUG_print("IMU Calibration: Minimum.\n"));

	return true;
}

//return true-->process ongoing
//return false-->process finished
bool Bearing_Monitor::IMU_Cal_Loop(bool completeCal){

	bool calibrated;

	// Exit if calibration is not in progress
	if (_IMU_cal_status != CAL_INPROGRESS) return false;

	// Loops until fully calibrated or calibration time exceeded
	if (_cal_iter++ < MAX_ITER) {
		// check=true performs complete initial calibration + check (system==3 required)
		// check=false ensures minimum recalibration after each power-on ( as long as mag and gyro are over 1, data is realiable)
		calibrated = (completeCal==true?_bno.isFullyCalibrated():getCalibrationStatus());
		 _IMU_cal_status= (calibrated? CAL_RESULT_RECALIBRATED: CAL_INPROGRESS);

		//refreshCalStatus();

	} else {
		// Calibration period exceeded
		// System status is lower than 3, IMU does not provide data of enough quality for calibration
		_IMU_cal_status= CAL_RESULT_NOT_CALIBRATED;
		displaySensorOffsets();
		DEBUG_print("\nCalibration time out!\n");
		DEBUG_print("WARNING: Calibration failed. Bearing values might be inaccurate.\n");
		return false;
	}

	if (_IMU_cal_status==CAL_RESULT_RECALIBRATED) {
		DEBUG_print("\nCalibrated! Ok\n");
		// Heading value is not received until a slight movement is detected by IMU
		// Practically speaking this is not an issue, but some info is provided to user
		_heading_isValid = false;
		DEBUG_print("Move slightly to start receiving IMU data\n");
		updateHeading();
		return true;
	}

	return true;
}


bool Bearing_Monitor::IMU_startCalCheck(int max_loop) {
	_cal_iter = max_loop;
	_IMU_check = CHECK_ONGOING;
	adafruit_bno055_offsets_t newCalib;

    //Only first iteration
    _bno.getSensorOffsets(newCalib);
	setIniCalib(newCalib); // Defines these offsets as the initial ones
	DEBUG_print("Calibration Results:\n");
    displaySensorOffsets(newCalib);
	DEBUG_print("\nCheck Sensor Orientation:\n");
	return true;
}

// _completeCal=true performs complete initial calibration + check (system==3 required)
// _completeCal=false ensures minimum recalibration after each power-on ( as long as mag and gyro are 3, data is realiable)
bool Bearing_Monitor::IMU_CalCheck_Loop(void){

	if (_IMU_check != CHECK_ONGOING) return false;
	//Loop
	if (_cal_iter-- > 0)
	{
		sensors_event_t event;
		_bno.getEvent(&event);

		_x = int(event.orientation.x);
		_y = int(event.orientation.y);
		_z = int(event.orientation.z);

		_IMU_check = CHECK_ONGOING;
	} else {

		_IMU_check = CHECK_FINISHED;
	}

	return (_IMU_check==CHECK_ONGOING);
}

bool Bearing_Monitor::getCheckXYZ (uint16_t &x, int8_t &y, uint8_t &z) {

	if (_IMU_check == CHECK_ONGOING) {
		x= _x;
		y= _y;
		z= _z;
		return true;
	} else {
		x= 0;
		y= 0;
		z= 0;

		return false; //fn only return valid values when check is ongoing.
	}
	return false;
}

bool Bearing_Monitor::getCheckSGAM(uint8_t &S, uint8_t &G, uint8_t &A, uint8_t &M){

	//if (_IMU_status != CAL_INPROGRESS) return false; //fn only return valid values when check is ongoing.
	S=_calSys;
	G=_calGyro;
	A=_calAccel;
	M=_calMagn;

	return true;
}

void Bearing_Monitor::displayCalStatus(void)
{
    DEBUG_print("\t");
    if (!_calSys)
    {
      DEBUG_print("! ");
    }

    /* Display the individual values */
	sprintf(DEBUG_buffer,"Sys:%i G:%i A:%i M:%i\n", _calSys, _calGyro, _calAccel, _calMagn);
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
}

void Bearing_Monitor::refreshCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    _bno.getCalibration(&system, &gyro, &accel, &mag);

    _calSys = system;
    _calGyro = gyro;
    _calAccel = accel;
    _calMagn = mag;

}



// updates heading even if data is not of sufficient quality
// return = true: accurate data; false= low data quality
bool Bearing_Monitor::updateHeading(){
	bool calStatus = true;
#ifdef SHIP_SIM
	_heading = _SIMheading;
	_heading_isValid = true;

#else
	calStatus = getCalibrationStatus();
	// - VECTOR_EULER         - degrees
	imu::Vector<3> euler = _bno.getVector(Adafruit_BNO055::VECTOR_EULER);
	float read_heading = euler.x();

	//even when heading is not valid, sometimes values can be used
    //if  (!_heading_isValid) {
		//v.2.5.B2 // IMU is not providing any value, keep previous value as the best approach
    	//if (int(_heading)!=0) _heading_isValid = true;
    	if (read_heading!=0) {
    		_heading_isValid = true;
    		_heading = read_heading;
    	}
    //}
#endif
	return calStatus;
}

bool Bearing_Monitor::updateHeading(bool valid, float HDM){
	if (valid) _heading = HDM;
	_heading_isValid = valid;
	return valid;
}

// fn available if system status value is not required
bool Bearing_Monitor::getCalibrationStatus(void) {
	uint8_t system;
	return getCalibrationStatus(system);
}

// return true if sensors are enough calibrated
// return false on the contrary
// Based on this solution to Technical Query, as long as mag and gyro are 3, data is realiable.
// https://community.bosch-sensortec.com/t5/MEMS-sensors-forum/BNO055-Calibration-Staus-not-stable/td-p/8375
// Due to difficulties in calibration, required sensor calibration status has been reducted.

bool Bearing_Monitor::getCalibrationStatus(uint8_t &system) {
	uint8_t gyro, accel, mag = 0;
	_bno.getCalibration(&system, &gyro, &accel, &mag);
	return (gyro>1 and mag>1);// (gyro==3 and mag==3); (system==3 and gyro==3 and mag==3);
}

void Bearing_Monitor::displaySensorOffsets()
{
	adafruit_bno055_offsets_t newCalib;
	_bno.getSensorOffsets(newCalib);
	displaySensorOffsets(newCalib);
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

//FUNCTIONAL MODULE:SHIP SIMULATOR
void Bearing_Monitor::SIM_updateShip(int tillerAngle) {
	static unsigned long DelayCalcStart = millis();

	if ((millis() -DelayCalcStart) < deltaT) return;

	//_SIMheading = 180; return;

	float unstat= random(unstat0)-unstat0/2;
	float alfa = (tillerAngle-unstat)*alfaMax/(sqrt(sq(float(tillerAngle-unstat))+softFactor));
	_SIMheading = _SIMheading *intertia + (1-intertia)*(_SIMheading + deltaT * alfa);
	_SIMheading = reduce360(_SIMheading);


	DelayCalcStart = millis();

}
