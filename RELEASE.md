# RELEASE


## Version: 4.1
## System requirements implemented
| Req.ID# | Description | Criticity | Implemented |

| SR-1 | Minimize cost of adquisition. Based on COTS components and integrate Open-source projects as possible | High | Yes |

| SR-2 | Ease integration with other systems or components by using industry standards. | High | Yes |

| SR-3 | Ease system adoption from developers and users with appropriate documentation, minimizing requirements for technical knowledge on electronics, physics or SW. | High | Partial. User guide available, pending installation and commissioning chapters |

| SR-4 | Modular approach priorizing escalability of components to implement requirements in later stages. Use of object oriented code. | High | Yes |

| SR-5 | Provide test means in order to ease testing and recording of results | High | Yes: Ship and linear actuator simulators available at compile time. Also external ship simulator available |

## Autopilot requirements implemented
| Req.ID# | Status | Description | Priority |

| AP-3 | Closed | Minimize power consumption | High |

| AP-4 | Closed | Internal compass tilt compensated | High |

| AP-6 | Closed | Calibration procedure for internal compass before installation supported by Autopilot without additional OEM SW. | High |

| AP-7 | Closed | Compensation of permanent deviation of installed compass with reference to the bow. | High |

| AP-15 | Closed | Capability to record required parameters to analyze AP performance for different external conditions for testing purposes.     | High           |

| AP-18 | Closed | USB electrical interface. | High |

| AP-19 | Closed | Alternative NMEA 0183 version 2.x or | High | er electrical interface (RS422). | Nice to have

| AP-20 | Closed | NMEA 0183 data protocol at least versions 2.x to 4.x (4800 baud, w/o parity, 8 bits,1 bit stop) | High

| AP-22 | Closed | Reception of minimum autopilot information required for Nav mode | High

| AP-24 | Closed | Transmision of internal compass magnetic course. | High

| AP-25 | Closed | Transmision of tiller position. | Low

| AP-27 | Closed | Stand-by mode provides user capability to command linear actuator changes in and out by short and long leaps. | High

| AP-28 | Closed | Compass mode. Autopilot will follow the current course. | High

| AP-31 | Closed | Nav mode. AP will bear to a Waypoint by following the required course received 
from an external source. | High

| AP-34 | Closed | User definable central position of tiller | High

| AP-35 | Closed | Adjust AP bearing during Autopilot mode by 1º or 10º | High

| AP-39 | Closed | Ensure safety of the operations by requiring user confirmation before proceed 
and/or sound warning (i.e. error, start, stop, tack, decission/ action of user required...). | High

| AP-40 | Closed | Minimum recomended motor controller performance: Bidirectional single-brushed. 
Support motor voltage to 16V. Support continous current: 10 A | High

| AP-41 | Closed | Voltage threshold: 10-16V | High

| AP-16   | Closed | Different deadband configurations for bad weather conditions: predefined, user defined, desactivated.   | High           |

|AP-17 | Closed | Automatic deadband configuration  | Nice to have |  

| AP-36 | Closed | Tack starboard/ portboard in Compass mode. AP will turn by 100º the current heading in the direction selected by the user. | High|

| AP-38 | Closed | Tack in Nav mode is not possible. | High |

| AP-5    | Closed | Capability to use as alternative an external compass  | Nice to have |  

| AP-21 | Closed | Reception of magnetic/real course from external compass | Nice to have |

| AP-29 | Closed | Wing mode. AP will maintain current Apparent Wind Angle. | Medium |

| AP-30 | Closed | Wind mode is only possible when wind information is available from an external source. | Medium |

## Requirements partially implemented
| Req.ID# | Status | Description | Priority |Limitations |

| AP-8  | Closed | Procedure to define installation parameters supported by Autopilot without additional SW. | High |IMU and linear actuator calibration functionalities from Android App. Predefined values for other parameters are workable|

| AP-11   | Closed | Predefined pilot gain configurations for different speeds and boats. | High  | One predefined set of PID gains is available at first installation. |

| AP-26 | Closed | Reception of real and apparent wind information. | High |

| AP-37 | Closed | Tack in Wind mode. AP will turn to head the same Real Wind Angle but on the oposite band. Turn direction will be selected by AP as the shortest path to reach the target heading. | High |

| AP-10   | Closed   | Autotrim of tiller to keep current heading when lateral forces (wind, tide, non-centered engine) require permanent correction of tiller to follow the current way-route. Maximum 1 minute for autotrimming. This is commonly obtained with I-integral component of PID adapted to peculiarities of the boat dynamics | High |

| AP-12   | Closed | Autogain for different SOW (speed over water).  | Low            |

| AP-13   | Closed | If SOW is not available, user configurable speed shall be used. | AP.-14  | Closed | User configurable gain for testing purposes.    | High           |

| AP-23 | Closed | Reception of SOW information from nautical slide. In Km or nautical miles. | Low |


# Autopilot requirements not implemented

| Req.ID# | Status | Description| Priority|

| AP-2    | Closed | Protection by design of mechanical and electronic parts from blockage, intense use, short-cirtuit and humidity. Parts exposed to weather conditions shall have IPX4 equivalent protection.                  | High         |

| AP-9    | Closed | Mounting at starboard or portboard | High           |

| AP-32 | Closed | Nav mode. If not available from external source, the required course will be BTW. | Nice to have |

# Autopilot requirements to be defined
| Req.ID# | Status | Description| Priority|

| AP-33 | Open | AP mode without rudder feedback. Central position of tiller is by default considered the position where Autopilot is set to Compass mode. PID feedback based on delta to CTS | Low |

| AP-42 | Open | Define AP limitations and installation specifications based on selected linear actuator specifications | High |
  
## Released versions
- V0.1 beta1 Initial version
- V0.1 beta2: Improves calibration information through App and NMEA I/F
- V2.0 beta1: Improves performance of Android App.  Compatible with Virtuino for Fenix V2.0
- V2.1 beta1: Provides additional information to CTS panel in Android App V2.1. Compatible with Virtuino for Fenix V2.0 and V2.1
- v.2.6.B1 implementation of capability to receive bearing from external IMU through HDM messages reception
- v.2.6.B1 implementation of capability to receive relative wind direction through VWR messages reception
- v.2.6.B2 IMU Calibration blocked in operational modes: IMU recalibration in ALL operational modes (not only STAND_BY)
- v.2.6.B2 IMU is not providing any value, keep previous value as the best approach
- v.2.6.B2 implementation of Wind Mode
- v.2.6.B2 Compatible with Virtuino for Fenix V3.0
- v.3.1.B1 Compatibility with Virtuino for Fenix App.4.1 (download Virtuino 6 App in Google Play Store) and retrocompatibility with Virtuino for Fenix App.3.0 (download Virtuino 5 App in Google Play Store)
- v.3.1.B1 Fix internal IMU issue in 2.6.B1 ( Internal IMU not working, external IMU ok)
- v.3.1.B1 Fix IMU recalibration in ALL operational modes: Fixes lost of autopilot operation when IMU recalibrates
- v.3.1.B1 Improved IMU validity data management. After 2 recalibrations w/o suceeding set STAND_BY
- v.3.1.B1 For developers: All config parameters into one file: Fenix_config.h
- v.3.2.B1 Autotune mode, proposes PID values based on boat performance test
- v.3.2.B1 Change of deadband min/MAX values: min: 1deg, max 5deg
- v.3.2.B1 Deadband mode button: select autodeadband mode (min, MAX, auto) from Virtuino for Fenix App.4.2
- v.3.2.B2 Heading deviation function is not applicable to External IMU
- v.3.2.B2 Fixed auto-calibration management
- v.3.2.B2 Deleted some debugging messages
- v.3.3B1 BNO055 IMU Library rebuilt. RTIMU open-source Library. 2 operational modes: internal or external fusion
- v.3.3B1 Additional IMU device available: Pololu MinIMU9V5
- v.3.3B1 VIRTUAL_ACTUATOR simulate rudder turn
- v.3.3B1 Fixed bug in NMEA DEC_COURSE_10 function
- v.3.3B1 Known limitations: 
- v.3.3B1 Known limitations: IMU must be installed: When IMU is not installed/found autopilot stops. It should raise a Warning and continue waiting for external compass information
- v.3.3B1 Known limitations: NMEA I/F: Centered Tiller Position and Heading alignment set parameter to 0 is not accepted
- v.3.4.B1 Additional IMU device available. Sparkfun 9DoF IMU Breakout - ICM-20948 (Qwiic) 
- v.3.4.B1 Known limitations:
- v.3.4.B1 Known limitations: Not compatible with BNO055
- v.3.4.B1 Known limitations: NMEA I/F: Centered Tiller Position and Heading alignment set parameter to 0 is not accepted
-v.3.5.B1
- Implementation of PID improvements: Derivative low-pass filter, Anti-windup, limit ITerm, reset ITerm
- Implementation of TESTER_IF to report internal time via Serial I/F
- Improved heading in External HDM mode
- v.3.6.B1
- NMEA I/F: Read relative wind and speed information
- Improvement of PID: Integrative and Derivative, adaptation to boat speed
- Remove spureous warning 5 (IMU requires calibration)
- v.4.0.B1
- First compatibility with new version of Fenix App.5.0
- Deprecated IMU: Only compatible with IMU Sparkfun ICM20948. Other IMU discontinued: Pololu MinIMU9V5
- Enhances configuration flexibility for installation parameters from Fenix App.
- Major refactor to Autopilot and related modules for improved handling of external heading, wind, and speed data.
- Adds support for HDG: true heading, magnetic variation, and heading deviation.
- v4.1.B1
- Updates interfaces and debug output to support new predictive features
- Refactors actuator control for smoother direction changes and speed ramping
- Adjusts configuration and rudder limits for improved control
- Info: TESTER_IF introduces predictive yaw delta calculation using IMU data for information only. Doesnt change PID logic
- Deprecated: Deprecates unused PID autotune application

