/*
 * HMIArq.h
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#ifndef HMIArq_H_
#define HMIArq_H_

#include "Autopilot.h"

// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"

enum e_operation {OP_ADD, OP_DEC, OP_MULT, OP_DIV};
enum e_requestStatus {NO_USER_REQUEST, WAITING_USER_ANSWER, USER_ACCEPTED, USER_REJECTED, REQUEST_TIME_OUT};

static e_requestStatus _requestStatus = NO_USER_REQUEST; // There is a request to user for confirmation to change Waypoint
static double _requestTime =0; // time when request was launched

class HMIArq {

public:
	HMIArq(Autopilot*);
	virtual ~HMIArq();

	//Library I/F
	virtual void setup() = 0;
	virtual void refresh() = 0;

protected:
	Autopilot* MyPilot;
	void Start_Stop(e_start_stop type=CURRENT_HEADING);
	void Start_Stop_wind(void);
	void Set_TrackMode(s_APB APB);
	void Inc_Rudder_1();
	void Inc_Rudder_10();
	void Dec_Rudder_1();
	void Dec_Rudder_10();
	void Stop_Rudder();
	void Inc_Course_1();
	void Inc_Course_10();
	void Inc_Course_100();
	void Dec_Course_1();
	void Dec_Course_10();
	void Dec_Course_100();

	void Inc_Mix_1();
	void Inc_Mix_10();
	void Dec_Mix_1();
	void Dec_Mix_10();
	void Set_NewCourse(float newCourse);
	void Set_NextCourse(float nextCourse);

	void Set_NextCourse_delta(int delta);
	void Set_Tacking(int delta);

	void Accept_Next(void);
	void Set_NewDeltaCourse(float newDCourse);
	void Set_Headalign();
	void Enter_Exit_FBK_Calib();
	void Start_Cancel_AutotunePID();


	void ResetPID();
	void Request_PIDgain(s_PIDgain & PIDgain);
	void Request_instParam(s_instParam & instParam);
	void Request_APinfo(s_APinfo & APinfo);
	void Request_IMUcal(s_IMUcal & IMUcal);
	void Request_FBKcal(s_FBKcal & FBKcal);

	void Change_PID(s_PIDgain_flag, s_gain);
	void Change_PID_rel (s_PIDgain_flag change, e_operation op, float value );
	void Apply_PIDrecom();
	bool Change_instParam (s_instParam instParam);
	void setDBConf (type_DBConfig status);
	void nextDBConf (void);
	void received_APB( s_APB APB);
	void received_HDM( s_HDM HDM);
	void received_SOG( s_SOG SOG);
	void received_VWR( s_VWR VWR);

	void Start_Cal(char sensor = '-');
	void Cal_NextSensor (void);
	void Cancel_Cal();
	void Save_Cal();
	void Save_instParam();
	void Save_PIDgain();
	void Save_HCParam(); // Saves GAIN and Inst.Param HARDCODED values
	void Load_calibrate_py(s_calibrate_py calibrate_py); // Load ICM20948 calibration parameters from calibrate.py programme

	bool check_userRequest(void);
	static e_requestStatus userRequest (void);
	static e_requestStatus userRequestAnswer (bool answer);
	static e_requestStatus getRequestStatus ();

private:
	void operation (e_operation op, float &K, float value);
	static bool updateRequestTimeout ();

};

#endif /* HMIArq_H_ */
