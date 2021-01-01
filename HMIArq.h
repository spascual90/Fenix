/*
 * HMIArq.h
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#ifndef HMIArq_H_
#define HMIArq_H_

//rudder inc/dec rate
#define RATE_1 10
#define RATE_10 30

// Maximum time (in millisecs) to answer to user request
#define MAX_USER_ANSWER_TIME 5000

enum e_operation {OP_ADD, OP_DEC, OP_MULT, OP_DIV};
enum e_requestStatus {NO_USER_REQUEST, WAITING_USER_ANSWER, USER_ACCEPTED, USER_REJECTED, REQUEST_TIME_OUT};

#include "Autopilot.h"

static e_requestStatus _requestStatus = NO_USER_REQUEST; // There is a request to user for confirmation to change Waypoint
//static char _requestDestID[5] ={'-','-','-','-','\0'}; // current request is associated to this WPID TODO: Avoid confirmation of wrong WPID
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
	void Set_TrackMode(s_APB APB);
	void Inc_Rudder_1();
	void Inc_Rudder_10();
	void Dec_Rudder_1();
	void Dec_Rudder_10();
	void Stop_Rudder();
	void Inc_Course_1();
	void Inc_Course_10();
	void Dec_Course_1();
	void Dec_Course_10();

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

	void ResetPID();
	void Request_PIDgain(s_PIDgain & PIDgain);
	void Request_instParam(s_instParam & instParam);
	void Request_APinfo(s_APinfo & APinfo);
	void Request_IMUcal(s_IMUcal & IMUcal);
	void Request_FBKcal(s_FBKcal & FBKcal);

	void Change_PID(s_PIDgain_flag, s_gain);
	void Change_PID_rel (s_PIDgain_flag change, e_operation op, float value );
	bool Change_instParam (s_instParam instParam);
	void setDBConf (type_DBConfig status);
	void received_APB( s_APB APB);

	void Start_Cal();
	void Cancel_Cal();
	void Save_Cal();
	void Save_instParam();
	void Save_PIDgain();
	void Save_HCParam(); // Saves GAIN and Inst.Param HARDCODED values

	bool check_userRequest(void);
	static e_requestStatus userRequest (void);
	static e_requestStatus userRequestAnswer (bool answer);
	static e_requestStatus getRequestStatus ();

private:
	void operation (e_operation op, float &K, float value);
	static bool updateRequestTimeout ();

};

#endif /* HMIArq_H_ */
