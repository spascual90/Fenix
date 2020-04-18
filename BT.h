#ifndef BT_H_
#define BT_H_

#include <Arduino.h>
#include "BTArq.h"
#include "HMIArq.h"

#define INFORMATION_SLOT (0)
#define WARNING_SLOT (10)
#define ERROR_SLOT (20)


class BT: public BTArq, public HMIArq {

public:
	BT(Autopilot*);
	virtual ~BT();


	//HMIArq I/F implementation
	void setup();
	void refresh();

private:
	Autopilot* MyPilot;
	void updateSpecialBT();
	s_PIDgain_flag _k_change;

};

#endif /* BT_H_ */
