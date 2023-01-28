/*
 * BearingMonitor.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "BearingMonitor.h"

//#include "GPSport.h" // Include this library to output to DEBUG_PORT

#ifdef BNO055_INTERNAL_FUSION
	#include "DevBNO055Int.h"
#endif
#ifdef BNO055_EXTERNAL_FUSION
	#include "DevBNO055Ext.h"
#endif
#ifdef MINIMU9V5
	#include "DevMinIMU9V5.h"
#endif

BearingMonitor::BearingMonitor(float headingDev = 0) {
	setHeadingDev( headingDev);
}

BearingMonitor::~BearingMonitor() {
	// TODO Auto-generated destructor stub
	_IMU_status = NOT_DETECTED;
}

e_IMU_status BearingMonitor::IMU_setup(long EE_address){

#ifdef SHIP_SIM
		_IMU_status = SIMULATED;
		_internalIMU_status= INT_NOT_DETECTED;

		return _IMU_status;
#endif

	#ifdef BNO055_INTERNAL_FUSION
	_imuDevice= (DevBNO055Int *) IMUDevice::createIMUDevice();    // create the imu_device object
#endif
#ifdef BNO055_EXTERNAL_FUSION
	_imuDevice= (DevBNO055Ext *) IMUDevice::createIMUDevice();    // create the imu_device object
#endif
#ifdef MINIMU9V5
	_imuDevice= (DevMinIMU9V5 *) IMUDevice::createIMUDevice();    // create the imu_device object
#endif

	if (_imuDevice->IMU_setup(EE_address)) {
		_internalIMU_status= INT_DETECTED;
		_IMU_status = CAL_MODE;
		_IMU_cal_status = CAL_START;
	}else {
		_IMU_status = NOT_DETECTED;
		_internalIMU_status= INT_NOT_DETECTED;
	}

	IBIT();
	return _IMU_status;
}

void BearingMonitor::IBIT(){

	_imuDevice->IBIT();
}


// Recurrent loop for calibration
//CAL_START-->CAL_INPROGRESS-->(CAL_RESULT_NOT_CALIBRATED; CAL_RESULT_RECALIBRATED)
//CAL_RESULT_RECALIBRATED-->completeCal?-->CHECK_ONGOING-->CHECK_FINISHED
//return true-->process ongoing
//return false-->process finished / IMU in operational mode again


e_IMU_cal_status BearingMonitor::EEload_Calib(long int &eeaddress){
	return (_imuDevice->EEload_Calib(eeaddress)?CAL_RESULT_RECALIBRATED:CAL_RESULT_NOT_CALIBRATED);
}

bool BearingMonitor::EEsave_Calib( long int &eeaddress){
	return (_imuDevice->EEsave_Calib(eeaddress));
}

bool BearingMonitor::compute_Cal_IMU(bool completeCal) {
	bool ret=false;
	//No calibration when IMU is in external mode.
	if (_IMU_status==EXTERNAL_IMU) return false;

	switch (_IMU_cal_status) {
	case CAL_NOT_STARTED:
		ret=false;
		break;
	case CAL_START:
		ret=IMU_startCalibration(completeCal);
		break;
	case CAL_INPROGRESS:
		ret=IMU_Cal_Loop(completeCal);
		break;
	//No action for other status
	default:
		break;
	}

	switch (_IMU_cal_status) {
	case CAL_RESULT_NOT_CALIBRATED:
		DEBUG_print("\nCalibration time out!\n");
		DEBUG_print("WARNING: Calibration failed. Bearing values might be inaccurate.\n");
		ret=false;
		break;
	case CAL_RESULT_RECALIBRATED:
		// Check calibration status
		// completeCal=true performs complete initial calibration + check (system==3 required)
		// completeCal=false ensures minimum recalibration after each power-on ( as long as mag and gyro are 3, data is realiable)
		if (_IMU_check == CHECK_NOT_STARTED) {
			_imuDevice->displaySensorOffsets();
			DEBUG_print("\nCalibrated! Ok\n");
			// Heading value is not received until a slight movement is detected by IMU
			// Practically speaking this is not an issue, but some info is provided to user
			DEBUG_print("Move slightly to start receiving IMU data\n");
			updateHeading();
			ret=false;
		}

		if (completeCal) {
			ret=compute_check_IMU();
			//DEBUG_print("CHECK\n");
		}
		break;

	//No action for other status
	default:
		break;
	}

	if (ret == false and _IMU_status != OPERATIONAL) {
		//DEBUG_print("***Finished calib\n");
		_IMU_status = OPERATIONAL;
		_IMU_cal_status = CAL_NOT_STARTED;
		_IMU_check = CHECK_NOT_STARTED;
	}

	return ret;
}

//return true-->process ongoing
//return false-->process finished
bool BearingMonitor::compute_check_IMU(void) {

	switch (_IMU_check) {
	case CHECK_NOT_STARTED:
		//start check
		return IMU_startCalCheck();
		break;
	case CHECK_ONGOING:
		return IMU_CalCheck_Loop();
		break;
	case CHECK_FINISHED:
		return false;
		break;
	}

	return (_IMU_check ==CHECK_ONGOING);
}


bool BearingMonitor::IMU_startCalibration(bool completeCal) {
	//_cal_iter = 0;
	_IMU_cal_status = CAL_INPROGRESS;

	//First iteration only
	DEBUG_print("Start IMU Calibration...\n");
	return _imuDevice->IMU_startCalibration(completeCal);
}

void BearingMonitor::reset_calibration () {
	//DEBUG_print("***reset calib\n");
	//IMU_setup();
	//resetSensorOffsets();
	//displaySensorOffsets();
}


bool BearingMonitor::getCheckXYZ (uint16_t &x, int8_t &y, uint8_t &z) {

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

bool BearingMonitor::getCheckSGAM(uint8_t &S, uint8_t &G, uint8_t &A, uint8_t &M){

	//if (_IMU_status != CAL_INPROGRESS) return false; //fn only return valid values when check is ongoing.
	S=_calSys;
	G=_calGyro;
	A=_calAccel;
	M=_calMagn;

	return true;
}

void BearingMonitor::displayCalStatus(void)
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

e_IMU_status BearingMonitor::updateHeading(bool changeSourceEnabled, bool validExternal, float HDMExternal){
#ifdef SHIP_SIM
	_heading = _SIMheading;
	_heading_isValid = true;
	_heading_isFrozen = false;
	return SIMULATED;
#else
	//decide if changing IMU status and source
	if (changeSourceEnabled and _IMU_status!=CAL_MODE) {
		if (validExternal) {
			_IMU_status = EXTERNAL_IMU;
			//DEBUG_print("***change to EXTERNAL_IMU\n");
		} else {
			if (_internalIMU_status == INT_DETECTED) {
				_IMU_status = OPERATIONAL;
				//DEBUG_print("***change to OPERATIONAL\n");
			} else {
				_IMU_status = NOT_DETECTED;
				//DEBUG_print("***change to NOT_DETECTED\n");
			}
		}
	}


	//update heading based on IMU status
	switch (_IMU_status) {
	case OPERATIONAL:
	case CAL_MODE:
		updateHeading();

		break;
	case EXTERNAL_IMU:
		updateHeading(validExternal, HDMExternal);

		break;
	case NOT_DETECTED:
		break;
	//case SIMULATED:
	//	break;
	}

	return _IMU_status;

#endif
}


void BearingMonitor::updateHeading(void){
	if (_IMU_status == EXTERNAL_IMU) return;

	_heading_isFrozen=false;
	_heading_isValid=true;
	_heading = _imuDevice->updateHeading();
}

e_IMU_status BearingMonitor::updateHeading(bool valid, float HDM){
	_heading_isValid = valid;
	_heading_isFrozen = false; //in external mode, there is no info of heading quality
	if (valid) _heading = HDM;
	return _IMU_status;
}

bool BearingMonitor::IMU_Cal_Loop(bool completeCal){

	// Exit if calibration is not in progress
	if (_IMU_cal_status != CAL_INPROGRESS) return false;
	bool ret = _imuDevice->IMU_Cal_Loop(completeCal);
	if (ret==false) _IMU_cal_status=CAL_RESULT_RECALIBRATED ;

	return ret;
}

bool BearingMonitor::IMU_startCalCheck(void) {
	//_cal_iter = max_loop;
	_IMU_check = CHECK_ONGOING;
	return true;
}

bool BearingMonitor::IMU_CalCheck_Loop(void){
	//Check funcionality not implemented
	_IMU_check = CHECK_FINISHED;
	return false;
}

void BearingMonitor::refreshCalStatus(void)
{
//    /* Get the four calibration values (0..3) */
//    /* Any sensor data reporting 0 should be ignored, */
//    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    _imuDevice->getCalibrationStatus(system, gyro, accel, mag);

    _calSys = system;
    _calGyro = gyro;
    _calAccel = accel;
    _calMagn = mag;
}

bool BearingMonitor::getCalibrationStatus(void) {
	refreshCalStatus();
	return (_calGyro>1 and _calMagn>1);// (gyro==3 and mag==3); (system==3 and gyro==3 and mag==3);
	return true;
}

//FUNCTIONAL MODULE:SHIP SIMULATOR
void BearingMonitor::SIM_updateShip(int tillerAngle) {
	static unsigned long DelayCalcStart = millis();

	if ((millis() -DelayCalcStart) < deltaT) return;

	float unstat= random(unstat0)-unstat0/2;
	float alfa = (tillerAngle-unstat)*alfaMax/(sqrt(sq(float(tillerAngle-unstat))+softFactor));
	_SIMheading = _SIMheading *intertia + (1-intertia)*(_SIMheading + deltaT * alfa);
	_SIMheading = reduce360(_SIMheading);


	DelayCalcStart = millis();

}
