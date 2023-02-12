#ifndef FENIX_CONFIG_H_

//Fenix version
#define ARDUINO_VERSION "v.3.3.B1"

// RELEASE v.2.0.B1
// RELEASE v.2.1.B1
//v.2.3.B1 implementation of capability to receive bearing from external IMU through HDM messages reception
//v.2.4.B1 implementation of capability to receive relative wind direction through VWR messages reception
//v.2.5.B2 IMU Calibration blocked in operational modes: IMU recalibration in ALL operational modes (not only STAND_BY)
//v.2.5.B2 IMU is not providing any value, keep previous value as the best approach
//v.2.6.B1 implementation of Wind Mode
// RELEASE v.2.6.B1
//v.3.0.B1 Compatibility with Virtuino 6 (Virtuino for Fenix App.4.0) and retrocompatibility with Virtuino 5 (Virtuino for Fenix App.3.0)
//v.3.1.B1 Fix IMU not working since 2.5 (internal IMU not working, external IMU ok)
//v.3.1.B1 Fix IMU recalibration in ALL operational modes: Fixes lost of autopilot operation when IMU recalibrates
//v.3.1.B1 Improved IMU validity data management. After 2 recalibrations w/o suceeding set STAND_BY
//v.3.1.B1 All config parameters into one file: Fenix_config.h
// RELEASE v.3.1.B1
//v.3.2.B1 Autotune mode, proposes PID values based on boat performance test
//v.3.2.B1 Change of deadband min/MAX values: min: 1deg, max 5deg
//v.3.2.B1 Deadband mode button: select autodeadband mode (min, MAX, auto) from Virtuino for Fenix App.4.2
//v.3.2.B1 Requires Virtuino App:
// RELEASE v.3.2.B1
//v.3.2.B2 Heading deviation function is not applicable to External IMU
//v.3.2.B2 Fixed auto-calibration management
//v.3.2.B2 Deleted some debugging messages
// RELEASE v.3.2.B2
//v.3.3B1 BNO055 IMU Library rebuilt. RTIMU open-source Library. 2 operational modes: internal or external fusion
//v.3.3B1 Additional IMU device available: Pololu MinIMU9V5

//DEBUG
// Defined for Release version
#define BUZZER //Comment this line to silent buzzer. SAFETY NOTICE: Only for DEBUGGING purposes!
#define TXNMEA //Comment this line to stop periodic NMEA Transmission

// Commented for Release version
//#define SHIP_SIM // Uncomment to simulate boat to tune PID
//#define VIRTUAL_ACTUATOR // Uncomment to simulate rudder at bow. Useful when linear actuator is not available
//#define RESTORE_EEPROM //Uncomment this line to reset EEPROM memory
//#define DEBUG

//Other libraries necessary for printing to serial monitor
#include "GPSport.h" // Some libraries requires to uncomment this line to print debug messages to serial monitor
//#include <simplot.h> //SIMPLOT FOR DEBUGGING PURPOSE ONLY


// PIN Configuration
#define PIN_RUDDER A8
#define PIN_RUDDER_NAME "A8"
#define PIN_BUZZER A12

#define PIN_PWM 6
#define PIN_DIR 7
//SDA - I2C data pin, connect to your microcontrollers I2C data line. This pin can be
//used with 3V or 5V logic, and there's a 10K pullup on this pin.
#define PIN_SDA 20
//SCL - I2C clock pin, connect to your microcontrollers I2C clock line. This pin can be
// used with 3V or 5V logic, and there's a 10K pullup on this pin
#define PIN_SCL 21



// *** BearingMonitor.h ***
//Only one IMU driver shall be defined at a time
#define MINIMU9V5
//#define BNO055_EXTERNAL_FUSION
//#define BNO055_INTERNAL_FUSION //Sensor fusion performed internally by BNO055.

//Define IMU axis orientation
# define IMU_ORIENTATION 0
// Valid values of IMU_ORIENTATION:
// IMU Device components on top 0 // Valid for MinIMU9V5
// IMU Device components on bottom 1 // Valid for MinIMU9V5

// *** Autopilot.h ***

#define DELAY_BUZZBEAT_TIME 50 // Buzzer beat time in msec.
#define MAX_APB_TIME 10000 // Maximum time of APB data validity
#define MAX_HDM_TIME 10000 // Maximum time of HDM data validity
#define MAX_VWR_TIME 10000 // Maximum time of VWR data validity
#define LONG_LOOP_TIME 100 // Loops to update current course, target bearing and calibration status
#define W_DISPLAY_TIME 2000 // Loops to display Warnings

// *** ActuatorController.h ***

	#define MIN_SPEED 0
	#define MAX_SPEED 255


// *** ActuatorManager.h ***

// ACTUATOR PARAMETERS
#define SPEED_CRUISE 255

// emcNMEA.h
# define DELAY_TX_TIME 1000 // period of NMEA transmission in millisecs (1000 = 1sec)
# define DELAY_TX1_TIME 5000 // period of PEMC transmission in millisecs (5000 = 3sec)

// autotune
# define ATUNE_NOISE 5 // magnetic degrees (input signal noise). Maximum bandwith value recommended
# define ATUNE_STEP 500 // rudder degrees (D value for output step)
# define ATUNE_LOOKBACK 50 //mseg look back time (local peaks filtering)

// *** DeadbandTrim.h ***

// Period definition
#define DELAY_SAMPLING_PERIOD 1000 //mSec long period for evaluation
#define NUMBER_SAMPLING 10 // number of iterations within long period for evaluation

#define VALUE_MAXDB 5//15 magnetic degrees
#define VALUE_MINDB 0//5 magnetic degrees
#define VALUE_MAXTRIM 15 // Rudder degrees

// *** RudderFeedback.h ***

// FEEDBACK PARAMETERS HARDCODED
// MIN/MAX VALUES FOR IDEAL LINEAR ACTUATOR
#define D_MIN_FEEDBACK 0 //AS READ FROM PIN_RUDDER IN THE RETRACTED POSITION OF THE ACTUATOR
#define D_MAX_FEEDBACK 1023 //AS READ FROM PIN_RUDDER IN THE EXTENDED POSITION OF THE ACTUATOR

#define MIN_ERROR_FEEDBACK 3
#define DEFAULT_ERROR_FEEDBACK 10 //A number between 0 and 1023 to protect linear actuator from overuse

// ADDITIONAL RUDDER PARAMETERS
// Parameters for rudder
#define DEFAULT_MRA 512 // MRA = ABS_MIN_RUDDER Value in degrees *10
#define RUDDER_LENGHT 1023 // Value in degrees *10

// VIRTUAL_ACTUATOR Parameters
#define VA_MRA 512 // Rudder value. MIN_RUDDER = -MRA; MAX_RUDDER = MRA-1
#define VA_ERROR 5
#define VA_DELTACENTEROFRUDDER 0
#define VA_MINFEEDBACK 34 //value between 0 and 1024
#define VA_MAXFEEDBACK 935 //value between 0 and 1024
#define VA_SPEEDFEEDBACK 0.05 // speed in mm/mseg    20mm/seg--> 0.02 mm/mseg
#define VA_LENGHTFEEDBACK 300.0 // mm lenght


// *** HMIArq.h ***

//rudder inc/dec rate
#define RATE_1 10
#define RATE_10 30

// Maximum time (in millisecs) to answer to user request
#define MAX_USER_ANSWER_TIME 5000


#endif
