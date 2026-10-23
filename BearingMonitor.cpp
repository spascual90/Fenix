/*
 * BearingMonitor.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "BearingMonitor.h"

#ifdef ICM20948
	#include "DevICM20948.h"
#endif

BearingMonitor::BearingMonitor(float magneticVariation = 0, float headingDev = 0) {
	setMagneticVariation(magneticVariation);
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

#else

#ifdef ICM20948
	_imuDevice= (DevICM20948 *) IMUDevice::createIMUDevice();    // create the imu_device object
#endif

	if (_imuDevice->IMU_setup(EE_address)) {
		_internalIMU_status= INT_DETECTED;
		if (_imuDevice->isExternalCalibration()){
			//This IMU requires external calibration
			_IMU_status = OPERATIONAL;
			_IMU_cal_status = CAL_NOT_STARTED;
		}else {
			// only for future capability to calibrate internally
			_IMU_status = CAL_MODE;
			_IMU_cal_status = CAL_START;
		}
	}else {
		_IMU_status = NOT_DETECTED;
		_internalIMU_status= INT_NOT_DETECTED;
	}

	IBIT();

#endif

	return _IMU_status;
}

void BearingMonitor::IBIT(){
	DEBUG_sprintf("Magnet.Var", getMagneticVariation());
	DEBUG_sprintf("Heading Dev.", getHeadingDev());
	_imuDevice->IBIT();
}


// Recurrent loop for calibration
//CAL_START-->CAL_INPROGRESS-->(CAL_RESULT_NOT_CALIBRATED; CAL_RESULT_RECALIBRATED)
//CAL_RESULT_RECALIBRATED-->completeCal?-->CHECK_ONGOING-->CHECK_FINISHED
//return true-->process ongoing
//return false-->process finished / IMU in operational mode again


e_IMU_cal_status BearingMonitor::EEload_Calib(long int eeaddress){

	return (_imuDevice->EEload_Calib(eeaddress)?CAL_RESULT_RECALIBRATED:CAL_RESULT_NOT_CALIBRATED);
}

bool BearingMonitor::EEsave_Calib( long int eeaddress){
	return (_imuDevice->EEsave_Calib(eeaddress));
}

bool BearingMonitor::compute_Cal_IMU(char sensor) {
	bool ret=false;
	//No calibration when IMU is in external mode.
	if (_IMU_status==EXTERNAL_IMU) return false;

	switch (_IMU_cal_status) {
	case CAL_NOT_STARTED:
		_IMU_cal_status = CAL_START;
		ret=true;
		break;
	case CAL_START:
		ret=IMU_startCalibration(sensor);
		break;
	case CAL_INPROGRESS:
		ret=IMU_Cal_Loop();
		break;
	//No action for other status
	default:
		break;
	}

	switch (_IMU_cal_status) {
	case CAL_RESULT_NOT_CALIBRATED:
		DEBUG_print(F("\nCalibration time out!\n"));
		DEBUG_print(F("WARNING: Calibration failed. Bearing values might be inaccurate.\n"));
		ret=false;
		break;
	case CAL_RESULT_RECALIBRATED:
		// Check calibration status
		// completeCal=true performs complete initial calibration + check (system==3 required)
		// completeCal=false ensures minimum recalibration after each power-on ( as long as mag and gyro are 3, data is realiable)
		if (_IMU_check == CHECK_NOT_STARTED) {
			//if (!_imuDevice->isExternalCalibration())
				//DEBUG_print(F("\nCalibrated! Ok\n"));
				// Heading value is not received until a slight movement is detected by IMU
				// Practically speaking this is not an issue, but some info is provided to user
				//DEBUG_print(F("Move slightly to start receiving IMU data\n"));
				//updateHeading();
			ret=false;
		}

		//TODO: Check calibration quality
		if (false) {
			ret=compute_check_IMU();
		}
		break;

	//No action for other status
	default:
		break;
	}

	if (ret == false and _IMU_status != OPERATIONAL) {
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


bool BearingMonitor::IMU_startCalibration(char sensor) {
	//_cal_iter = 0;
	_IMU_cal_status = CAL_INPROGRESS;

	//First iteration only
	DEBUG_print(F("Start IMU Calibration...\n"));
	return _imuDevice->IMU_startCalibration(sensor);
}

void BearingMonitor::reset_calibration (long &eeAddress) {
	DEBUG_print(F("***reset calib\n"));
		//ID
	    long ID =0;
	    EEPROM.put(eeAddress, ID);
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
    DEBUG_print(F("\t"));
    if (!_calSys)
    {
      DEBUG_print(F("! "));
    }

    /* Display the individual values */
	sprintf(DEBUG_buffer,"Sys:%i G:%i A:%i M:%i\n", _calSys, _calGyro, _calAccel, _calMagn);
	DEBUG_print(DEBUG_buffer);
	DEBUG_flush();
}

// True = Magnetic + Magnetic Variation (ie. earth zone)
// Magnetic = Compass + Magnetic Deviation (eg. heading deviation, soft/hard magnet)
//headingExternal: Compass heading
//headingT: True heading
e_IMU_status BearingMonitor::updateHeading(unsigned long NMEA_RXtime){
#ifdef SHIP_SIM
	_heading = _SIMheading;
	_heading_isValid = true;
	_heading_isFrozen = false;
	return SIMULATED;
#else
	static bool sb_ext_reset = true;
	//update heading based on IMU status
	switch (_IMU_status) {
	case OPERATIONAL:
	case CAL_MODE:
		updateHeadingINT();
		sb_ext_reset = true;
		break;

	case EXTERNAL_IMU:
		if (NMEA_RXtime!=0) {
			updateHeadingEXT(NMEA_RXtime, sb_ext_reset);
			sb_ext_reset= false;
		}
		break;

	case NOT_DETECTED:
		break;
	}

	return _IMU_status;

#endif
}

// function for internal IMU only
// if (_IMU_status == EXTERNAL_IMU) use updateHeading(float HDT, unsigned long HDT_RXtime, bool firstTime)
void BearingMonitor::updateHeadingINT(void){
	_heading_isFrozen=false;
	_heading_isValid=true;
	_heading =  _imuDevice->updateHeading();
	update_RateOfTurn();

#ifdef DEBUG_SIMPLOT_HEADING
	static int a=0;
	a++;
	if (a==10) {
		plot1(NeoSerial, _heading);
		a=0;
	}

#endif
	updateHeadingT();


}

// Function for external IMU only. For internal IMU use updateHeading(void) instead
e_IMU_status BearingMonitor::updateHeadingEXT(unsigned long HDT_RXtime, bool firstTime){
	static float prev_ext_heading=0;
	static float pendiente =0;
	static unsigned long prev_tiempo = 0;
	static unsigned long prev_HDT_RXtime = 0;

	_heading_isValid = true;
	_heading_isFrozen = false; //in external mode, there is no info of heading quality

	if (HDT_RXtime==0) return _IMU_status;

	unsigned long tiempo = millis();

	if (firstTime) {
		//DEBUG_print("First time\n");
		_heading = _ext_heading_dt;
		prev_ext_heading=_ext_heading_dt;
		prev_tiempo = HDT_RXtime;
		prev_HDT_RXtime = HDT_RXtime;
		pendiente=0;
	}

	if (prev_HDT_RXtime!=HDT_RXtime) {
		//ajustamos heading al valor recibido
		_heading = _ext_heading_dt;
		//Calculamos pendiente cada vez que se reciba un nuevo mensaje HDT
		double deltaHDT = deltaAngle(prev_ext_heading, _ext_heading_dt);
		pendiente = ((deltaHDT)*1000.0)/(HDT_RXtime-prev_HDT_RXtime);
		prev_ext_heading = _ext_heading_dt;
		prev_HDT_RXtime = HDT_RXtime;

	} else {
		// Extrapolamos heading con la última pendiente calculada
		float delta_heading = (pendiente*(tiempo-prev_tiempo))/1000.0;
		// limitamos delta a algo físicamente razonable
		delta_heading = fmin(delta_heading, 15.0);
		_heading += delta_heading;
		_heading = reduce360(_heading);
		prev_tiempo = tiempo;
	}

	update_RateOfTurn(pendiente);
	updateHeadingT();

	return _IMU_status;
}

bool BearingMonitor::IMU_Cal_Loop(void){

	// Exit if calibration is not in progress
	if (_IMU_cal_status != CAL_INPROGRESS) return false;
	bool ret = _imuDevice->IMU_Cal_Loop();
	if (ret==false) _IMU_cal_status=CAL_RESULT_RECALIBRATED;

	return ret;
}

void BearingMonitor::Cal_NextSensor(void){
	_imuDevice->Cal_NextSensor();
}

bool BearingMonitor::isExternalCalibration(void){
	return _imuDevice->isExternalCalibration();
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
//	DEBUG_print(F("refreshCalStatus\n"));
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


// calibrate.py external programme to calibrate A and M
bool BearingMonitor::set_calibrate_py_offsets(float B[3], float Ainv[3][3], char sensor) {
	//ICM20948AHRS_setOffsets(G_offset[3]);
	_imuDevice->set_calibrate_py_offsets(B, Ainv, sensor);
	if (sensor=='G') _calGyro=3;
	if (sensor=='A') _calAccel=3;
	if (sensor=='M') _calMagn=3;
	if (_calGyro + _calAccel +_calMagn == 9) _calSys ==3;
	return true;
}

//FUNCTIONAL MODULE:SHIP SIMULATOR

void BearingMonitor::SIM_updateShip(int tillerAngle) {
	static unsigned long DelayCalcStart = millis();

	if ((millis() -DelayCalcStart) < deltaT) return;

	float unstat= 0;//random(unstat0)-unstat0/2;

	float alfa = (tillerAngle-unstat)*alfaMax/(sqrt(sq(float(tillerAngle-unstat))+softFactor));
	_SIMheading = _SIMheading *intertia + (1-intertia)*(_SIMheading + deltaT * alfa);
	_SIMheading = reduce360(_SIMheading);


	DelayCalcStart = millis();

}

// WAVE MODULE
void BearingMonitor::setup_WaveObserver() {
    waveCfg.Tnomoto = 5.0f;
    waveCfg.Knomoto= 0.25f;
    waveCfg.Tbias = 120.0f;

    // Initial wave period 6 s.
    waveCfg.omega0Initial = TWO_PI / 6.0f;
    waveCfg.omega0Min = TWO_PI / 15.0f;
    waveCfg.omega0Max = TWO_PI / 2.0f;
    waveCfg.zetaWave = 0.10f;

    waveCfg.kPsi = 0.08f;
    waveCfg.kR = 0.04f;
    waveCfg.kBias = 0.001f;
    waveCfg.waveGain = 0.6f;

    // Important requested configuration variable:
    // true  => use the reliable rIMU measurement in the observer.
    // false => ignore rIMU and use only heading + model.
    waveCfg.useRimu = true;
    waveCfg.kRimuWave = 0.5f;
    waveCfg.kRimuLF = 0.1f;

    waveCfg.enableAdaptiveOmega = true;
    waveCfg.alphaOmega = 0.02f;
    waveCfg.minWaveAmplitude = radians(0.5f);
    waveCfg.zeroCrossHysteresis = radians(0.4f);
    waveCfg.minWavePeriod = 2.0f;
    waveCfg.maxWavePeriod = 15.0f;

    waveCfg.innovationLimit = radians(10.0f);
    waveCfg.rateInnovationLimit = radians(20.0f);
    waveCfg.maxWaveYaw = radians(15.0f);
    waveCfg.maxWaveRate = radians(25.0f);
    waveCfg.maxLFRate = radians(15.0f);
    waveCfg.maxBias = radians(5.0f);

    waveCfg.waveRmsAlpha = 0.01f;

    waveObserver.begin(waveCfg);

    waveObserver_reset();
}

void BearingMonitor::waveObserver_reset(void){
	waveObserver.reset(
			radians(convert360to180(getCurrentHeadingT())),
			radians(get_RateOfTurn())
	);
}

void BearingMonitor::loop_WaveObserver(bool allowAdapt, float deltaRudder) {// , float Teff, float Keff) { // bool, deg

    const unsigned long now = micros();
    float dt = (now - lastMicros) * 1.0e-6f;
    lastMicros = now;

    // Optional: enforce a sane dt if loop stalls.
    if (dt <= 0.0f || dt > 0.5f) {
        return;
    }

    const float yaw_rad = radians(convert360to180(getCurrentHeadingT()));
    const float ROT_rad = radians(get_RateOfTurn());

    // Sobrescribe con valores experimentales reales
    //waveObserver.setBoatYawModel(Teff, Keff);
    //waveObserver.setBoatYawModel(0.42f, 0.0722f);

    // Since waveCfg.useRimu = true, this call will use rIMU.
    // If useRimu were false, rIMU would be ignored.
    //DEBUG_sprintf("yaw,ROT,rudd", degrees(yaw_rad), degrees(ROT_rad), deltaRudder);

    waveObserver.update(
    	yaw_rad,
		ROT_rad,
        radians(deltaRudder),
        allowAdapt,
        dt
    );

    // These are the signals to feed into the autopilot PID/reference controller.
    //waveObserver.getPsiLF();
    //waveObserver.getRLF();

#ifdef DEBUG_WAVE
    // Debug at low rate.
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        lastPrint = millis();

        (allowAdapt?DEBUG_print(F("Adapting...\n")):DEBUG_print(F("Still.\n")));
        DEBUG_sprintf("T,K", waveObserver.getTnomoto(), waveObserver.getKnomoto());
        DEBUG_sprintf("psiRaw_deg", float (degrees(yaw_rad)));
        DEBUG_sprintf("psiLF_deg", getHeadingTforControl());
        DEBUG_sprintf("rLF_deg_s", get_angularSpeedforControl());
        DEBUG_sprintf("psiWave_deg", float (degrees(waveObserver.getPsiWave())));
        DEBUG_sprintf("wavePeriod_s", waveObserver.getWavePeriod());
        DEBUG_sprintf("conf", waveObserver.getWaveConfidence());
        DEBUG_sprintf("e_deg", waveObserver.getInnovation());
        DEBUG_sprintf("eR_deg_s", float (degrees(waveObserver.getRateInnovation())));
    }
#endif

}

// Sobrescribe con valores experimentales reales
void BearingMonitor::setBoatYawModel(float Teff, float Keff) {
	waveObserver.setBoatYawModel(Teff, Keff);
}


float BearingMonitor::getHeadingTforControl(void) {
	return convert180to360(degrees(waveObserver.getPsiLF()));
}

float BearingMonitor::get_angularSpeedforControl(void) {
	return waveObserver.getRLF();
}




