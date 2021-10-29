# RELEASE
## Version: 2.6
## System requirements implemented
| Req.ID# | Description | Criticity | Implemented |
| ------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------- | ----------- |
| SR-1 | Minimize cost of adquisition. Based on COTS components and integrate Open-source projects as possible | High | Yes |
| SR-2 | Ease integration with other systems or components by using industry standards. | High | Yes |
| SR-3 | Ease system adoption from developers and users with appropriate documentation, minimizing requirements for technical knowledge on electronics, physics or SW. | High | Partial. User guide available, pending installation and commissioning chapters |
| SR-4 | Modular approach priorizing escalability of components to implement requirements in later stages. Use of object oriented code. | High | Yes |
| SR-5 | Provide test means in order to ease testing and recording of results | High | Yes: Ship and linear actuator simulators available at compile time |

## Autopilot requirements implemented
| Req.ID# | Status | Description | Priority |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------- |
| AP-3 | Closed | Minimize power consumption | High |
| AP-4 | Closed | Internal compass tilt compensated | High |
| AP-6 | Closed | Calibration procedure for internal compass before installation supported by Autopilot without additional OEM SW. | High |
| AP-7 | Closed | Compensation of permanent deviation of installed compass with reference to the bow. | High |
| AP-18 | Closed | USB electrical interface. | High |
| AP-19 | Closed | Alternative NMEA 0183 version 2.x or | High | er electrical interface (RS422). | Nice to have
| AP-20 | Closed | NMEA 0183 data protocol at least versions 2.x to 4.x (4800 baud, w/o parity, 8 bits,1 bit stop) | High
| AP-22 | Closed | Reception of minimum autopilot information required for Nav mode | High
| AP-24 | Closed | Transmision of internal compass magnetic course. | High
| AP-25 | Closed | Transmision of tiller position. | Low
| AP-27 | Closed | Stand-by mode provides user capability to command linear actuator changes in and out by short and long leaps. | High
| AP-28 | Closed | Compass mode. Autopilot will follow the current course. | High
| AP-31 | Closed | Nav mode. AP will bear to a Waypoint by following the required course received from an external source. | High
| AP-34 | Closed | User definable central position of tiller | High
| AP-35 | Closed | Adjust AP bearing during Autopilot mode by 1º or 10º | High
| AP-39 | Closed | Ensure safety of the operations by requiring user confirmation before proceed and/or sound warning (i.e. error, start, stop, tack, decission/ action of user required...). | High
| AP-40 | Closed | Minimum recomended motor controller performance: Bidirectional single-brushed. Support motor voltage to 16V. Support continous current: 10 A | High
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
| ------- | ------ | ----------------------------------------------------------------------------------------- | -------- |------ |
| AP-8  | Closed | Procedure to define installation parameters supported by Autopilot without additional SW. | High |IMU and linear actuator calibration functionalities from Android App. Predefined values for other parameters are workable|
| AP-11   | Closed | Predefined pilot gain configurations for different speeds and boats. | High  | One predefined set of PID gains is available at first installation. |
| AP-26 | Closed | Reception of real and apparent wind information. | High |
AP-37 | Closed | Tack in Wind mode. AP will turn to head the same Real Wind Angle but on the oposite band. Turn direction will be selected by AP as the shortest path to reach the target heading. | High |

# Autopilot requirements not implemented

| Req.ID# | Status | Description                                                                                                                                                                                                 | Priority       |
| ------- | ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------ |
| AP-2    | Closed | Protection by design of mechanical and electronic parts from blockage, intense use, short-cirtuit and humidity. Parts exposed to weather conditions shall have IPX4 equivalent protection.                  | High         |
| AP-9    | Closed | Mounting at starboard or portboard | High           |
| AP-12   | Closed | Autogain for different SOW (speed over water).  | Low            |
| AP-13   | Closed | If SOW is not available, user configurable speed shall be used. | AP.-14  | Closed | User configurable gain for testing purposes.    | High           |
| AP-15   | Closed | Capability to record required parameters to analyze AP performance for different external conditions for testing purposes.     | High           |
AP-23 | Closed | Reception of SOW information from nautical slide. In Km or nautical miles. | Low |
  | AP-32 | Closed | Nav mode. If not available from external source, the required course will be BTW. | Nice to have |

# Autopilot requirements to be defined
| Req.ID# | Status | Description                                                                                                                                                                                                 | Priority       |
| ------- | ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------ |
| AP-10   | Open   | Autotrim of tiller to keep current heading when lateral forces (wind, tide, non-centered engine) require permanent correction of tiller to follow the current way-route. Maximum 1 minute for autotrimming. | To be cheked |     |     |     |     |
  | AP-33 | Open | Central position of tiller is by default considered the position where Autopilot is set to Compass mode. | Low |
  | AP-42 | Open | Maximum thrust torque (at the end of the tiller): 250N. Minimum angular speed (tiller): 0 Kg: TBD º/sec 35 mm/seg 20 Kg: TBD º/sec 30 mm/seg 40 Kg: TBD º/sec 20 mm/seg | High |
