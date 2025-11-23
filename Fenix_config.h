#ifndef FENIX_CONFIG_H_

//Fenix version
#define ARDUINO_VERSION "v.3.6.B1"

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
// RELEASE v.3.2.B2
//v.3.2.B2 Heading deviation function is not applicable to External IMU
//v.3.2.B2 Fixed auto-calibration management
//v.3.2.B2 Deleted some debugging messages
// RELEASE v.3.3.B1
//v.3.3B1 BNO055 IMU Library rebuilt. RTIMU open-source Library. 2 operational modes: internal or external fusion
//v.3.3B1 Additional IMU device available: Pololu MinIMU9V5
//v.3.3B1 VIRTUAL_ACTUATOR simulate rudder turn
//v.3.3B1 Fixed bug in NMEA DEC_COURSE_10 function
// Known limitations
// IMU must be installed: When IMU is not installed/found autopilot stops. It should raise a Warning and continue waiting for external compass information
// NMEA I/F: Centered Tiller Position and Heading alignment set parameter to 0 is not accepted
// Heading to bow cannot be used as it causes overflow of heading over 360
// RELEASE v.3.4.B1
//v.3.4.B1 Additional IMU device available: Sparkfun ICM20948
// Known limitations
// Not compatible with BNO055
// NMEA I/F: Centered Tiller Position and Heading alignment set parameter to 0 is not accepted
//v.3.4.B1.1 correct B1 errors:
// PEMC12 message after magnetometer offset was wrong and not recognized by Fenix_Cal
// Linear actuator error check message was too long
//v.3.4.B2 CORRECT B1.1 errors:
// EEPROM address of IMU overlapping InstParam.
//v.3.5.B1
// Implementation of PID improvements: Derivative low-pass filter, Anti-windup, limit ITerm, reset ITerm
// Implementation of TESTER_IF to report internal time via Serial I/F
// Improved heading in External HDM mode
//v.3.6.B1
// NMEA I/F: Read relative wind and speed information
// Improvement of PID: Integrative and Derivative, adaptation to boat speed
// Remove spureous warning 5 (IMU requires calibration)
//v.4.0.B1
// First compatibility with new version of Fenix App.5.0
// Enhances configuration flexibility for installation parameters.
// Major refactor to Autopilot and related modules for improved handling of external heading, wind, and speed data.
// Adds support for HDG: true heading, magnetic variation, and heading deviation, and updates data structures and method signatures to reflect these changes.
// Updates Bluetooth interface, and improves millis-based timing for external data validity checks.


//DEBUG
// Defined for Release version
#define BUZZER //Comment this line to silent buzzer. SAFETY NOTICE: Only for DEBUGGING purposes!
#define TXNMEA //Comment this line to stop periodic NMEA Transmission
// Commented for Release version
//#define PRINT_FREE_MEM
//#define SHIP_SIM // Uncomment to simulate boat to tune PID
#define VIRTUAL_ACTUATOR // Uncomment to simulate rudder. Useful when linear actuator is not available
//#define RESTORE_EEPROM //Uncomment this line to reset EEPROM memory
//#define DEBUG
#define TESTER_IF
//#define FREQ_MONITOR

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
//Only one IMU driver shall be defined at a time.
// Uncomment #define as applicable
//#define MINIMU9V5
#define ICM20948
//#define BNO055_EXTERNAL_FUSION
//#define BNO055_INTERNAL_FUSION //Sensor fusion performed internally by BNO055.

// For MPU9250 and BNO055: additional configuration required in RTIMULibDefs.h
//  IMU enable defs - only one should be enabled, the rest commented out
// BNO055_INTERNAL_FUSION or BNO055_EXTERNAL_FUSION: #define BNO055_28


//TODO: Integrate BNO055 IMU axis orientation options


//Define IMU axis orientation (MinIMU9V5 only)
# define IMU_ORIENTATION 0 // IMU Device components on top
// Valid values of IMU_ORIENTATION:
// IMU Device components on top 0 // Valid for MinIMU9V5 only
// IMU Device components on bottom 1 // Valid for MinIMU9V5 only

// For BNO055: Uncomment #define in RTIMULibDefs.h as applicable
//	RTIMU_XNORTH_YEAST--> BNO055 Device components on the bottom
//	RTIMU_XWEST_YNORTH--> BNO055 Device components on the top

// *** Autopilot.h ***

#define DELAY_BUZZBEAT_TIME 50 // Buzzer beat time in msec.
#define MAX_APB_TIME 10000 // Maximum time of APB data validity
#define MAX_HDTG_TIME 10000 // Maximum time of HDT/HDG data validity
#define MAX_SOG_TIME 10000 // Maximum time of SOG data validity
#define MAX_VWR_TIME 10000 // Maximum time of VWR data validity
#define LONG_LOOP_TIME 100 // Loops to update current course, target bearing and calibration status
#define W_DISPLAY_TIME 2000 // Loops to display Warnings
#define AVDSPEED 5
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

// *** RudderFeedback.h ***

// FEEDBACK PARAMETERS HARDCODED
// MIN/MAX VALUES FOR IDEAL LINEAR ACTUATOR
#define D_MIN_FEEDBACK 0 //AS READ FROM PIN_RUDDER IN THE RETRACTED POSITION OF THE ACTUATOR
#define D_MAX_FEEDBACK 1023 //AS READ FROM PIN_RUDDER IN THE EXTENDED POSITION OF THE ACTUATOR

#define MIN_ERROR_FEEDBACK 3
#define DEFAULT_ERROR_FEEDBACK 10 //A number between 0 and 1023 to protect linear actuator from overuse

// ADDITIONAL RUDDER PARAMETERS
// Parameters for rudder
//Changed default from 512. 35.0 is the standard maximum rudder angle used.
#define DEFAULT_MRA 350  // MRA = ABS_MIN_RUDDER Value in degrees *10
#define RUDDER_LENGHT 699//1023 // Value in degrees *10

// VIRTUAL_ACTUATOR Parameters
#define VA_MRA 350 //512 // Rudder value. MIN_RUDDER = -MRA; MAX_RUDDER = MRA-1
#define VA_ERROR 5
#define VA_DELTACENTEROFRUDDER 0
#define VA_MINFEEDBACK 34 //value between 0 and 1024
#define VA_MAXFEEDBACK 935 //value between 0 and 1024
#define VA_SPEEDFEEDBACK 0.040//0.027 // speed in mm/mseg    20mm/seg--> 0.02 mm/mseg
#define VA_LENGHTFEEDBACK 300.0 // mm lenght --> 7.5 seg to get to the center


// *** HMIArq.h ***

//rudder inc/dec rate
#define RATE_1 10
#define RATE_10 30

// Maximum time (in millisecs) to answer to user request
#define MAX_USER_ANSWER_TIME 5000



#endif
