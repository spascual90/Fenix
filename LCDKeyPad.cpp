/*
 * LCDKeyPad.cpp
 *
 *  Created on: 16 abr. 2017
 *      Author: Sergio
 */

#include "LCDKeyPad.h"

LCDKeyPad::LCDKeyPad(Autopilot* Pilot)
:HMIArq(Pilot)
{
	// TODO Auto-generated constructor stub
}

LCDKeyPad::~LCDKeyPad() {
	// TODO Auto-generated destructor stub
}

void LCDKeyPad::setup() {
	//Set the characters and column numbers.
	begin(16, 2);
	clear();
	pinMode(LCD_LIGHT, OUTPUT);

	turnLight(ON);
}

void LCDKeyPad::refresh() {
	//State = NO_BTN every loop cycle.
	int state = NO_BTN;

	//Refresh the button pressed.
	state = getButtonPressed();

	if (state!=NO_BTN) {turnLight(ON);
	  } else if (HMILightTimer()==false) {turnLight(OFF);}

	  if (_Delay==true) {
		  if ( millis() -_DelayStart <= _DelayTime) {

			  // DETECT DOUBLECLICK /////////////////////////
			  if ((state == lastState) and (millis() -_DelayStart >= DOUBLECLICK)) {
				  	  if (state == BTN_RIGHT && currentMenuItem==MENU_CHANGE_RUDDER) {
			 	     	 //If rudder right
			 	     	 //selectInstruction(INC_RUDDER_10);
				  		 Inc_Rudder_10();
			 	     	 printInstr(INC_RUDDER_10);
			 	      } else if (state == BTN_LEFT && currentMenuItem==MENU_CHANGE_RUDDER) {
			 	     	 //If ruddeer left
			 	     	 //selectInstruction(DEC_RUDDER_10);
			 	    	 Dec_Rudder_10();
			 	     	 printInstr(DEC_RUDDER_10);

			 	      } else if (state == BTN_RIGHT
			 	    		  && currentMenuItem==MENU_CHANGE_BEARING
							  && MyPilot->getCurrentMode()==AUTO_MODE )
			 	      {
			 		     //If bearing right

			 		     //selectInstruction(INC_COURSE_10);
			 		     Inc_Course_10();
			 		     printInstr(INC_COURSE_10);
			 		  } else if (state == BTN_LEFT
			 				  && currentMenuItem==MENU_CHANGE_BEARING
							  && MyPilot->getCurrentMode()==AUTO_MODE )
			 		  {
			 		     //If bearing left
			 		   	//selectInstruction(DEC_COURSE_10);
			 		   	Dec_Course_10();
			 		   	printInstr(DEC_COURSE_10);
			 		  }
					   	printTitle();

						//Save the last button pressed to compare.
						lastState = state;
						//Small delay
						delay(5);
			 	   }
			  return;
		  } else{
			  _Delay=false;
		  }
	  }


	   // DETECT CLICK ///////////////////////////////
	   if (state != lastState) {

	      if (state == BTN_UP) {
	         //If Up
	          changeMenu(&currentMenuItem, -1);
	      } else if (state == BTN_DOWN) {
	         //If Down
	    	  changeMenu(&currentMenuItem, +1);

	      } else if (state == BTN_SELECT && currentMenuItem==MENU_START_STOP) {
	         //If Selected
	    	 //selectInstruction(START_STOP);
	    	 Start_Stop();
	    	 printInstr(START_STOP);

	      } else if (state == BTN_RIGHT && currentMenuItem==MENU_CHANGE_RUDDER) {
	     	 //If rudder right
	     	 //selectInstruction(INC_RUDDER_1);
	    	  Inc_Rudder_1();
	    	  printInstr(INC_RUDDER_1);
	      } else if (state == BTN_LEFT && currentMenuItem==MENU_CHANGE_RUDDER) {
	     	 //If ruddeer left
	     	 //selectInstruction(DEC_RUDDER_1);
	    	  Dec_Rudder_1();
	    	  printInstr(DEC_RUDDER_1);

	      } else if ((MyPilot->getCurrentMode()==STAND_BY)
	    		  && (currentMenuItem==MENU_CHANGE_BEARING)
				  && (state == BTN_LEFT or state == BTN_RIGHT)
	      ) {
	     	printInstr(NOT_AVAILABLE);

	      } else if (state == BTN_RIGHT && currentMenuItem==MENU_CHANGE_BEARING) {
		     //If bearing right

		     //selectInstruction(INC_COURSE_1);
	    	  Inc_Course_1();
	    	  printInstr(INC_COURSE_1);
		  } else if (state == BTN_LEFT && currentMenuItem==MENU_CHANGE_BEARING) {
		     //If bearing left
		   	 //selectInstruction(DEC_COURSE_1);
			  Dec_Course_1();
			  printInstr(DEC_COURSE_1);
		  } else {
			  if (currentMenuItem == MENU_CHANGE_RUDDER) {
			     	 //selectInstruction(STOP_RUDDER);
				  Stop_Rudder();
			  }

	          displayMenu(currentMenuItem);
		  }
	   }

	   	printTitle();

		//Save the last button pressed to compare.
		lastState = state;

		//Small delay
	//	delay(5);
	}

void LCDKeyPad::displayMenu(int x) {
	//Display Menu Option based on Index.
	  setCursor(0,1);
	     switch (x) {
	      case MENU_CHANGE_RUDDER:
	    	 //CHANGE RUDDER COURSE
	        print ("+/- RUDDER    ");
	        break;
	      case MENU_START_STOP:
	    	  //START/STOP FOLLOW BEARING

	        print ("START/STOP    ");
	        break;
	       case MENU_CHANGE_BEARING:
	    	   // CHANGE TARGET BEARING

	        print ("+/- BEARING      ");
	        break;
	}
}

void LCDKeyPad::printTitle() {
  setCursor(0,0);
  float heading = MyPilot->getCurrentHeading();
  float targetBearing = MyPilot->getTargetBearing();

  if (MyPilot->getCurrentMode()==AUTO_MODE) {
	  	  	    print0 (heading);
	  	  	    print (heading, 1);
 	        	print (" -> ");
	  	  	    print0 (targetBearing);
	  	  	    print (targetBearing, 1);
 	        	print ("      ");
 	        } else if (MyPilot->getCurrentMode()==STAND_BY) {
 	        	print ("STAND BY ");
	  	  	    print0 (heading);
	  	  	    print (heading, 1);
 	        	print ("   ");
 	        } else{
 	        	print (MyPilot->getCurrentModeStr());
 	        	print ("   ");

 	        }

  	printCalibrationStatus();
	printRudderStatus();

}

void LCDKeyPad::printInstr(e_actions x) {
  setCursor(0,1);
  switch (x) {
  	  case START_STOP:
  		  return;
		  break;

     case INC_RUDDER_1:
       print ("RUDDER + 1    ");
       break;

     case DEC_RUDDER_1:
       print ("RUDDER - 1    ");
       break;

     case INC_COURSE_1:
       print ("COURSE + 1    ");
       break;

     case DEC_COURSE_1:
       print ("COURSE - 1    ");
       break;

     case INC_RUDDER_10:
       print ("RUDDER + 10    ");
       break;

     case DEC_RUDDER_10:
       print ("RUDDER - 10    ");
       break;

     case INC_COURSE_10:
       print ("COURSE + 10    ");
       break;

     case DEC_COURSE_10:
       print ("COURSE - 10    ");
       break;

     case NOT_AVAILABLE:
       print ("FN NOT AVAILABLE");
       break;
     default:
    	 break;
       }


  HMIDelay(1000);
}

void LCDKeyPad::HMIDelay(long mSec) {
	_Delay=true;
	_DelayStart = millis();
	_DelayTime = mSec;
}

void LCDKeyPad::changeMenu(int* MenuItem, int Delta){
	*MenuItem+=Delta;
	//If we are out of bounds on the menu then reset it.
	  if (*MenuItem < MENU_MIN)  {
	   *MenuItem = MENU_MIN;
	  }

	  if (*MenuItem > MENU_MAX) {
	   *MenuItem = MENU_MAX;
	  }
}

void LCDKeyPad::printCalibrationStatus() {
	setCursor(15,0);
	print ((MyPilot->Bearing_MonitorArq::getCalibrationStatus()?3:0));
}

void LCDKeyPad::printRudderStatus() {
	setCursor(15,1);
	int status = MyPilot->getRudderStatus();

	switch (status) {
	case CENTERED:
		print ("-");
	break;

	case EXTENDED:
		print (">");
	break;

	case RETRACTED:
		print ("<");
	break;
	}
}

void LCDKeyPad::print0(float heading) {

	if (heading <10) {print ("0");}
	if (heading <100) {print ("0");}
}
