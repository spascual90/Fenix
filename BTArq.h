#ifndef BTARQ_H_
#define BTARQ_H_

#include <Arduino.h>
#include <VirtuinoCM.h>                           // Include VirtuinoCM library to your code
#include "GPSport.h" // Ports configuration

void onReceived(char variableType, uint8_t variableIndex, String valueAsText);
String onRequested(char variableType, uint8_t variableIndex);

class BTArq: public VirtuinoCM {

public:
	BTArq(uint8_t max_V, uint8_t max_DV, uint8_t BTN_V);
	virtual ~BTArq();
	void vDelay(int delayInMillis);

protected:
	float *_V;           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
	float *_DV;

	virtual void updateBT(void) = 0;
	virtual void triggerAction (void) = 0;

	uint8_t getButtonPressed();
	void virtuinoRun();
	const uint8_t BT_NO_BTN = -1;

private:
	uint8_t _lastButton = BT_NO_BTN;

	boolean debug = false;              // set this variable to false on the final code to decrease the request time.

};


#endif /* BTARQ_H_ */
