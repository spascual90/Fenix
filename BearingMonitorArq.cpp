/*
 * BearingMonitor.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "BearingMonitorArq.h"

#include "GPSport.h" // Include this library to output to DEBUG_PORT

Bearing_MonitorArq::Bearing_MonitorArq(float headingDev = 0) {
	setHeadingDev( headingDev);
}

Bearing_MonitorArq::~Bearing_MonitorArq() {
	// TODO Auto-generated destructor stub
	_IMU_status = NOT_DETECTED;
}

void Bearing_MonitorArq::IBIT(){

	//DEBUG_print("IMU int... Started\n");
	DEBUG_print("!Compilation without IMU. Only external IMU\n");
	DEBUG_print(DEBUG_buffer);
	DEBUG_PORT.flush();
	//TODO: include routine to wait until bearing data is available (except calib IMU mode) While Sys<3 loop if not in x secs-->ask for calibration.
}


// Recurrent loop for calibration
//CAL_START-->CAL_INPROGRESS-->(CAL_RESULT_NOT_CALIBRATED; CAL_RESULT_RECALIBRATED)
//CAL_RESULT_RECALIBRATED-->completeCal?-->CHECK_ONGOING-->CHECK_FINISHED
//return true-->process ongoing
//return false-->process finished / IMU in operational mode again


e_IMU_status Bearing_MonitorArq::IMU_setup(long EE_address){

	#ifdef SHIP_SIM
		_IMU_status = SIMULATED;
		_internalIMU_status= INT_NOT_DETECTED;

#else
	IMU_setup_specific(EE_address);
#endif
	return _IMU_status;
}


e_IMU_cal_status Bearing_MonitorArq::EEload_Calib(long int &eeaddress){
	return EEload_Calib_specific(eeaddress);
}
bool Bearing_MonitorArq::EEsave_Calib( long int &eeaddress){
	return EEsave_Calib_specific(eeaddress);

}

bool Bearing_MonitorArq::compute_Cal_IMU(bool completeCal) {
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
		displaySensorOffsets();
		DEBUG_print("\nCalibration time out!\n");
		DEBUG_print("WARNING: Calibration failed. Bearing values might be inaccurate.\n");
		ret=false;
		break;
	case CAL_RESULT_RECALIBRATED:
		// Check calibration status
		// completeCal=true performs complete initial calibration + check (system==3 required)
		// completeCal=false ensures minimum recalibration after each power-on ( as long as mag and gyro are 3, data is realiable)
		if (_IMU_check == CHECK_NOT_STARTED) {
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
bool Bearing_MonitorArq::compute_check_IMU(void) {

	switch (_IMU_check) {
	case CHECK_NOT_STARTED:
		//start check
		return IMU_startCalCheck(CAL_CHECK_LOOP);
		break;
	case CHECK_ONGOING:
		//return Bearing_MonitorArq::IMU_CalCheck_Loop();
		return IMU_CalCheck_Loop();
		break;
	case CHECK_FINISHED:
		return false;
		break;
	}

	return (_IMU_check ==CHECK_ONGOING);
}


bool Bearing_MonitorArq::IMU_startCalibration(bool completeCal) {
	_cal_iter = 0;
	_IMU_cal_status = CAL_INPROGRESS;

	//First iteration only
	DEBUG_print("Start IMU Calibration...\n");
	return IMU_startCalibration_specific(completeCal);
}

void Bearing_MonitorArq::reset_calibration () {
	//DEBUG_print("***reset calib\n");
	//IMU_setup();
	//resetSensorOffsets();
	//displaySensorOffsets();
}


bool Bearing_MonitorArq::getCheckXYZ (uint16_t &x, int8_t &y, uint8_t &z) {

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

bool Bearing_MonitorArq::getCheckSGAM(uint8_t &S, uint8_t &G, uint8_t &A, uint8_t &M){

	//if (_IMU_status != CAL_INPROGRESS) return false; //fn only return valid values when check is ongoing.
	S=_calSys;
	G=_calGyro;
	A=_calAccel;
	M=_calMagn;

	return true;
}

void Bearing_MonitorArq::displayCalStatus(void)
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

e_IMU_status Bearing_MonitorArq::updateHeading(bool changeSourceEnabled, bool validExternal, float HDMExternal){
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

e_IMU_status Bearing_MonitorArq::updateHeading(bool valid, float HDM){
	_heading_isValid = valid;
	_heading_isFrozen = false; //in external mode, there is no info of heading quality
	if (valid) _heading = HDM;
	return _IMU_status;
}

// fn available if system status value is not required
bool Bearing_MonitorArq::getCalibrationStatus(void) {
	uint8_t system;
	return getCalibrationStatus(system);
}

//FUNCTIONAL MODULE:SHIP SIMULATOR
void Bearing_MonitorArq::SIM_updateShip(int tillerAngle) {
	static unsigned long DelayCalcStart = millis();

	if ((millis() -DelayCalcStart) < deltaT) return;

	float unstat= random(unstat0)-unstat0/2;
	float alfa = (tillerAngle-unstat)*alfaMax/(sqrt(sq(float(tillerAngle-unstat))+softFactor));
	_SIMheading = _SIMheading *intertia + (1-intertia)*(_SIMheading + deltaT * alfa);
	_SIMheading = reduce360(_SIMheading);


	DelayCalcStart = millis();

}
