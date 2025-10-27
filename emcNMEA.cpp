//  Copyright (C) 2014-2017, SlashDevin
//
//  This file is part of NeoGPS
//
//  NeoGPS is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  NeoGPS is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with NeoGPS.  If not, see <http://www.gnu.org/licenses/>.

// The first field of a sentence is called the "tag" and normally consists of a two-letter talker ID followed by a three-letter type code.
// DATA OVERRUN: took too long to print NMEA data! --> TIP: Autopilot might be receiving messages without TAG

#include "emcNMEA.h"
#include <MemoryFree.h>


emcNMEA::emcNMEA ()
:NMEAGPS()
{
	bufferStream= new StringStream ((String &)output);

}

void emcNMEA::sentenceOk()
{
	// Terminate the last field with a comma if the parser needs it.
	if (comma_needed()) {
		comma_needed( false );
	    chrCount++;
	    parseField(',');
	}
	// Order is valid
	INorder.isValid = YES;

	reset();

}


//----------------------------------------------------------------
// There was something wrong with the sentence.

void emcNMEA::sentenceInvalid()
{

  // All the values are suspect.  Start over.
  //DEBUG_print("sentenceInvalid\n");

  INorder.reset();
  nmeaMessage = NMEA_UNKNOWN;
  reset();
}

//----------------------------------------------------------------
// Process one character of an NMEA GPS sentence.

emcNMEA::decode_t emcNMEA::decode( char c )
{
	decode_t res = NMEAGPS::decode (c);
	switch (res) { //DECODE_CHR_INVALID, DECODE_CHR_OK, DECODE_COMPLETED

		case DECODE_CHR_OK:
			break;
		case DECODE_CHR_INVALID:
			//TIP if sentence: Remove CR, NL or CR/NL options from Serial Terminal

			sentenceInvalid(); break;
		case DECODE_COMPLETED:
			sentenceOk(); break;
	}
  return res;

} // decode


//----------------------------------------------------------------

bool emcNMEA::parseField(char chr)
{
  if (nmeaMessage >= (nmea_msg_t) EXT_FIRST_MSG) {
	//sprintf(DEBUG_buffer, "parseField:%i\n", nmeaMessage);
	//DEBUG_print();
    switch (nmeaMessage) {
      //Initial entry point for all PEMC messages.
      //Parse messages without parameters (eg.00, 09, 10)
      //Classify messages
      case PEMC_00: return classifyPEMC( chr );

      //After classification, message assigned to the right parser
      case PEMC_01: return parsePEMC_01( chr ); //Change rudder
      case PEMC_02: return parsePEMC_02( chr ); //Change CTS
      case PEMC_03:
    	  INorder.set_order(GET_INST);
    	  return parsePEMC_0304( chr ); //Get installation parameters
      case PEMC_04:
    	  INorder.set_order(SET_INST);
    	  return parsePEMC_0304( chr ); //Set installation parameters
      case PEMC_05:
    	  INorder.set_order(GET_GAIN);
    	  return parsePEMC_0506( chr ); //Get PID Gain
      case PEMC_06:
    	  INorder.set_order(SET_GAIN);
    	  return parsePEMC_0506( chr ); //Set PID Gain
      case PEMC_07:
    	  INorder.set_order(GET_APINFO);
    	  return parsePEMC_07( chr ); //Get AP Information
      case PEMC_08: return parsePEMC_08( chr ); //Request Information
      case PEMC_09: return parsePEMC_09( chr ); //Start IMU Calibration
      case PEMC_11: return parsePEMC_11( chr ); //Save
      case PEMC_14: return parsePEMC_14( chr ); //IMC20948 calibration values
      default: 
        break;
    }

  } else

    // Delegate
    return NMEAGPS::parseField(chr);


  return true;

} // parseField
// $GPAPB,A,A,0.0,L,N,V,V,043.6,,5,043.6,T,43.583042528455735,T*46
// $ECAPB,A,A,0.00,L,N,V,V,312.23,M,001,312.34,M,312.34,M*2A
// $GPAPB,A,A,0.00,R,N,V,,291.3,T,3,291.3,T,291.3,T*50

// $--APB,A,A,x.x,a,N,A,A,x.x  ,a  ,c--c,x.x  ,a,x.x               ,a*hh<CR><LF>
bool emcNMEA::parseAPB( char chr )
{
	bool ok = true;

	switch (fieldIndex) {
	  case 1: return ok;
	  case 2: return ok;
	  case 3: parseFloat( INorder.APB.XTE, chr, 2, INorder.APB.flag.XTE); break;// XTE distance value
	  case 4: parseDirSteer(chr); break;  // XTE dir: R/L
	  case 5: parseXTEUnits(chr); break; // // XTE distance units: N or K. Transforms XTE distance into N if necessary
	  case 6: parseAlarmCircle( chr ); break;// Alarm: A - Activated, other - no alarm.
	  case 7: parseAlarmPerp( chr ); break;// Alarm: A - Activated, other - no alarm
	  case 8: parse360(INorder.APB.BOD, chr); break;  // Angle: Magnitude, Reference (M/T)
	  case 9: parseAngleRef(INorder.APB.BOD, chr); break;
	  case 10: parseWPID( INorder.APB.destID, chr ); break;// WP ID: Up to 4 characters, rest ignored
	  case 11: parse360( INorder.APB.BTW, chr); break;
	  case 12: parseAngleRef(INorder.APB.BTW, chr); break;
	  //APB received by AvNav=12 decimals and OpenCPN=2 decimals
	  //numbesr below 13 decimals will be valid
	  case 13:parse360( INorder.APB.CTS, chr);  break; // Angle: Magnitude, Reference (M/T)
	  case 14: parseAngleRef(INorder.APB.CTS, chr);
		  INorder.APB.isValid=YES;
	  	  INorder.set_order(REQ_TRACK);
	  	  break;
	          }
	return ok;
}

// TODO: HDM obsolete. Parse HDG instead
// $--HDM,x.x,M*hh<CR><LF>
// Example: $IIHDM,245.0,M*21
bool emcNMEA::parseHDM( char chr )
{
	bool ok = true;
	switch (fieldIndex) {
	  case 1: parseFloat( INorder.HDM.HDM, chr, 2, INorder.HDM.flag.HDM); break; //Heading magnetic value
	  case 2:
		  INorder.HDM.isValid=YES;
	  	  INorder.set_order(EXT_HEADING);
		  break;
	          }
	return ok;
}

// TODO: Parse VHW or RMC for speed through water
//RMC Sentence NMEA 4.1:
// $GPRMC,125656.00,V,,,,,3.00,,260925,,,N*69
// $GPRMC,125656.00,V,,,,,,,260925,,,N*74
// $--RMC,hhmmss.ss,A,ddmm.mm,a,dddmm.mm,a,x.x,x.x,xxxx  ,x.x,a,m*hh (NMEA 2.3)
// $GPRMC,hhmmss.ss,A,ddmm.mm,a,dddmm.mm,a,4.5,x.x,xxxx  ,x.x,a,m,s*3C
// SOG = 4.5 Knots
bool emcNMEA::parseRMC( char chr )
{
	bool ok = true;

	switch (fieldIndex) {

//Field Number:
//1. UTC of position fix, hh is hours, mm is minutes, ss.ss is seconds.
//2. Status, A = Valid, V = Warning
//3. Latitude, dd is degrees. mm.mm is minutes.
//4. N or S
//5. Longitude, ddd is degrees. mm.mm is minutes.
//6. E or W
	case 1:
	case 2:
	case 3:
	case 4:
	case 5:
	case 6:
		return ok;
//7. Speed over ground, knots
	case 7:
		parseFloat( INorder.SOG.SOG, chr, 2, INorder.SOG.flag.SOG); break;// SOG value
//8. Track made good, degrees true
//9. Date, ddmmyy
//10. Magnetic Variation, degrees
//11. E or W
//12. FAA mode indicator (NMEA 2.3 and later)
//13. Nav Status (NMEA 4.1 and later) A=autonomous, D=differential, E=Estimated, M=Manual input mode N=not valid, S=Simulator, V = Valid
	case 8:
	case 9:
	case 10:
	case 11:
		return ok;
	case 12:
		  INorder.SOG.isValid=YES;
	  	  INorder.set_order(SOG);
		  break;
    }
return ok;
}
// TODO: Parse MWV instead of obsolete VWR
// Obsolete
// $--VWR,x.x,a,,,,,,*hh<CR><LF>
// Example: $IIVWR,045.0,L,12.6,N,6.5,M,23.3,K*52
bool emcNMEA::parseVWR( char chr )
{
	bool ok = true;
	switch (fieldIndex) {
	  case 1: parseFloat( INorder.VWR.windDirDeg, chr, 2, INorder.VWR.flag.windDirDeg); break; //Wind direction magnitude in degrees
	  case 2: parseWindDir(chr); break;  // Wind direction Left/Right of bow: R/L
	  case 3: return ok; //Speed
	  case 4: return ok; //N = Knots
	  case 5: return ok; //Speed
	  case 6: return ok; //M = Meters Per Second
	  case 7: return ok; //Speed
	  case 8: //K = Kilometers Per Hour
		  // TODO: If last field is empty Message is not recognized
		  INorder.VWR.isValid=YES;
	  	  INorder.set_order(RELATIVE_WIND);
		  break;
	          }
	return ok;
}
//$GPVTG,,,,,3.00,,,,N*2D
//$GPVTG,,,,,,,,,N*30
//$--VTG,x.x,T,x.x,M,x.x,N,x.x,K,m*hh<CR><LF> (new version NMEA 2.3)
bool emcNMEA::parseVTG( char chr )
{
	bool ok = true;
	switch (fieldIndex) {
	  case 1: return ok;
	  case 2: return ok;
	  case 3:
	  case 4:
	  case 5: parseFloat( INorder.SOG.SOG, chr, 2, INorder.SOG.flag.SOG); break;// SOG value
	  case 6:
	  case 7:
	  case 8:
	  case 9:
	  	  INorder.SOG.isValid=YES;
	  	  INorder.set_order(SOG);
		  break;
	          }
	return ok;
}
bool emcNMEA::classifyPEMC( char chr )
{
  bool ok = true;
  static bool EMC1x=false;

  switch (fieldIndex) {
  case 1:
    // The first field is actually a message subtype
	if (chrCount == 0) {
    	switch (chr) {
    		case '0': EMC1x=false; ok = true; break;// Message is PEMC,0#,...
    		case '1': EMC1x=true; ok = true; break;// Message is PEMC,1#...
    		default : ok = false;
    	}// end switch chr
    } // end ChrCount==0
    if (chrCount == 1) {
		if (!EMC1x) {
//			case false: // Message is PEMC,0#,...
				switch (chr) {
							// $PEMC,00
							case '0': nmeaMessage = (nmea_msg_t) PEMC_00; INorder.set_order(START_STOP); break;
							// $PEMC,01 to 09
							case '1': nmeaMessage = (nmea_msg_t) PEMC_01; break;
							case '2': nmeaMessage = (nmea_msg_t) PEMC_02; break;
							case '3': nmeaMessage = (nmea_msg_t) PEMC_03; break;
							case '4': nmeaMessage = (nmea_msg_t) PEMC_04; break;
							case '5': nmeaMessage = (nmea_msg_t) PEMC_05; break;
							case '6': nmeaMessage = (nmea_msg_t) PEMC_06; break;
							case '7': nmeaMessage = (nmea_msg_t) PEMC_07; break;
							case '8': nmeaMessage = (nmea_msg_t) PEMC_08; break;
							case '9': nmeaMessage = (nmea_msg_t) PEMC_09; break;
								//INorder.set_order(START_CAL);	break;
							default : ok = false;
				}// end switch chr
			} else {// case true: // Message is PEMC,1#,...
				switch (chr) {
							// $PEMC,10
							case '0': nmeaMessage = (nmea_msg_t) PEMC_10; INorder.set_order(EE_FBK_CAL); break;
							// $PEMC,11
							case '1': nmeaMessage = (nmea_msg_t) PEMC_11; break;
							// $PEMC,14
							case '4': nmeaMessage = (nmea_msg_t) PEMC_14; break;

							default : ok = false;
				} // end switch chr
		} // end Switch (EMC1x)

	// end ChrCount==1
    }
    if (chrCount!=0 and chrCount!=1){// chrCount > 1
        ok = (chr == ',');
    } // end if chrCount
  } // end case fieldindex


  return ok;

} // parsePEMC_00

bool emcNMEA::parsePEMC_01( char chr )
{
    switch (fieldIndex) {
        case 2:
            switch (chrCount) {
              case 0:
                if (chr != ',') {
                  	  switch (chr) {
                  	  case 'i':
                  		  INorder.set_order(INC_RUDDER_1);

                  		  break;
                  	  case 'I':
                  		  INorder.set_order(INC_RUDDER_10);

                  		  break;
                  	  case 'r':
                  		  INorder.set_order(DEC_RUDDER_1);

                  		  break;
                  	  case 'R':
                  		  INorder.set_order(DEC_RUDDER_10);

                  		  break;
                  	  }


                  	  }
                break;
            }

    }

  return true;

} // parsePEMC_01

bool emcNMEA::parsePEMC_02( char chr )
{
    switch (fieldIndex) {
        case 2:
            switch (chrCount) {
              case 0:
                if (chr != ',') {
                  	  switch (chr) {
                  	  case 'i':
                  		  INorder.set_order(INC_COURSE_1);

                  		  break;
                  	  case 'I':
                  		  INorder.set_order(INC_COURSE_10);

                  		  break;
                  	  case 'r':
                  		  INorder.set_order(DEC_COURSE_1);

                  		  break;
                  	  case 'R':
                  		  INorder.set_order(DEC_COURSE_10);

                  		  break;
                  	  case 'P': // Tack Portboard
                  		  INorder.set_order(DEC_COURSE_100);

                  		  break;
                  	  case 'S': // Tack Stardboard
                  		  INorder.set_order(INC_COURSE_100);

                  		  break;

                  		  break;
                  	  }


                  	  }
                break;
            }

    }

  return true;

} // parsePEMC_02

bool emcNMEA::parsePEMC_0304( char chr )
{

    switch (fieldIndex) {
        case 2: parseIntAuto( INorder.instParam.centerTiller , chr, INorder.instParam.flag.centerTiller ); break;
        case 3: parseIntAuto( INorder.instParam.maxRudder , chr, INorder.instParam.flag.maxRudder); break;
        case 4: parseIntAuto( INorder.instParam.avgSpeed , chr, INorder.instParam.flag.avgSpeed ); break;
        case 5: parseSide( chr, INorder.instParam.flag.instSide ); break;
        case 6: parseIntAuto( INorder.instParam.rudDamping , chr, INorder.instParam.flag.rudDamping ); break;
        case 7: parse180(INorder.instParam.magVariation, chr, INorder.instParam.flag.magVariation); break;
        case 8: parse180(INorder.instParam.headAlign, chr, INorder.instParam.flag.headAlign); break;
        case 9:
        	parseIntAuto( INorder.instParam.offcourseAlarm , chr, INorder.instParam.flag.offcourseAlarm );
        	INorder.instParam.isValid=YES;

        	break;
    }

  return true;

} // parsePEMC_0304

bool emcNMEA::parsePEMC_0506( char chr )
{

    switch (fieldIndex) {
        case 2: parseFloat( INorder.PIDgain.gain.Kp, chr, 2, INorder.PIDgain.flag.gain.Kp ); break;
        case 3: parseFloat( INorder.PIDgain.gain.Ki, chr, 2, INorder.PIDgain.flag.gain.Ki ); break;
        case 4: parseFloat( INorder.PIDgain.gain.Kd, chr, 2, INorder.PIDgain.flag.gain.Kd ); break;
        case 5: parseIntAuto(INorder.PIDgain.sTime, chr, INorder.PIDgain.flag.sTime  ); break;
        case 6:
        	parseDeadbandType( chr );
        	INorder.PIDgain.isValid=YES;

        	break;

    }

  return true;

} // parsePEMC_0506

bool emcNMEA::parsePEMC_07( char chr )
{

    switch (fieldIndex) {
    	case 2: parseAPmode( chr ); break;
    	case 3: parseInt( INorder.APinfo.rudder , chr ); break;
    	case 4:	parse360( INorder.APinfo.HDM, chr); break;
    	case 5: parse360( INorder.APinfo.CTS, chr); break;
    	case 6: parseFloat( INorder.APinfo.deadband , chr , 2, INorder.APinfo.flag.deadband ); break;
        case 7:
        	//parseFloat( INorder.APinfo.trim , chr , 2, INorder.APinfo.flag.trim );
        	INorder.APinfo.isValid=YES;

        	break;
    }

  return true;

} // parsef

bool emcNMEA::parsePEMC_08( char chr )
{
    switch (fieldIndex) {
        case 2:
            switch (chrCount) {
              case 0:
                if (chr != ',') {
                  	  switch (chr) {
                  	  case 'I':
                  		  INorder.set_order(REQ_INST);
                  		  INorder.isRequest= true;
                  		  break;
                  	  case 'G':
                  		  INorder.set_order(REQ_GAIN);
                  		  INorder.isRequest= true;
                  		  break;
                  	  case 'A': //AUTOPILOT INFO
						  INorder.set_order(REQ_INFO);
						  INorder.isRequest= true;
						  break;
                  	  }


                  	  }
                break;
            }

    }
  return true;
}
bool emcNMEA::parsePEMC_09( char chr )
{
    switch (fieldIndex) {
        case 2:
            switch (chrCount) {
              case 0:
                if (chr != ',') {
                  	  switch (chr) {
                  	  case 'G':
                  		  INorder.set_order(CAL_GYRO);
                  		  INorder.isRequest= true;
                  		  break;
                  	  case 'A':
                  		  INorder.set_order(CAL_ACCEL);
                  		  INorder.isRequest= true;
                  		  break;
                  	  case 'M':
						  INorder.set_order(CAL_MAGNET);
						  INorder.isRequest= true;
						  break;
                  	  case '-':
						  INorder.set_order(CAL_ALL);
						  INorder.isRequest= true;
						  break;
                  	  }


                  	  }
                break;
            }

    }
  return true;
}

bool emcNMEA::parsePEMC_11( char chr )
{
    switch (fieldIndex) {
        case 2:
            switch (chrCount) {
              case 0:
                if (chr != ',') {
                  	  switch (chr) {
                  	  case 'I':
                  		  INorder.set_order(SAVE_INST);
                  		  INorder.isRequest= true;
                  		  break;
                  	  case 'G':
                  		  INorder.set_order(SAVE_GAIN);
                  		  INorder.isRequest= true;
                  		  break;
                  	  case 'C':
						  INorder.set_order(SAVE_CAL);
						  INorder.isRequest= true;
						  break;
                  	  case 'R':
						  INorder.set_order(SAVE_HC);
						  INorder.isRequest= true;
						  break;
                  	  }


                  	  }
                break;
            }

    }
  return true;
}
//$PEMC,14,-120.03,3.26,31.86,3.33673,-0.09850,0.04508,-0.09850,3.21739,-0.02705,0.04508,-0.02705,3.25455,M*79
//$PEMC,14,288.12,249.14,-37.53,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,0.00000,G*7D
bool emcNMEA::parsePEMC_14( char chr )
{

    switch (fieldIndex) {

    	case 2: parseFloat( INorder.calibrate_py.GAM_B.x , chr, 2 ); break;
    	case 3: parseFloat( INorder.calibrate_py.GAM_B.y , chr, 2 ); break;
    	case 4: parseFloat( INorder.calibrate_py.GAM_B.z , chr, 2 ); break;
    	case 5:	parseFloat( INorder.calibrate_py.GAM_Ainv.m11, chr, 5); break;
    	case 6:	parseFloat( INorder.calibrate_py.GAM_Ainv.m12, chr, 5); break;
    	case 7:	parseFloat( INorder.calibrate_py.GAM_Ainv.m13, chr, 5); break;
    	case 8:	parseFloat( INorder.calibrate_py.GAM_Ainv.m21, chr, 5); break;
    	case 9: parseFloat( INorder.calibrate_py.GAM_Ainv.m22, chr, 5); break;
    	case 10: parseFloat( INorder.calibrate_py.GAM_Ainv.m23, chr, 5); break;
    	case 11: parseFloat( INorder.calibrate_py.GAM_Ainv.m31, chr, 5); break;
    	case 12: parseFloat( INorder.calibrate_py.GAM_Ainv.m32, chr, 5); break;
    	case 13: parseFloat( INorder.calibrate_py.GAM_Ainv.m33, chr, 5); break;
    	case 14:
        	  switch (chr) {
        	  case 'G':
        		  INorder.calibrate_py.sensor='G';
        		  break;
        	  case 'A':
        		  INorder.calibrate_py.sensor='A';
        		  break;
        	  case 'M':
        		  INorder.calibrate_py.sensor='M';
        		  break;
        	  }
    		  INorder.calibrate_py.isValid=YES;
    		  INorder.set_order(LOAD_calibrate_py);
    		  break;
    }

  return true;
}
//---------------------------------------------
bool emcNMEA::parseFix( char chr )
{
  if (chrCount == 0) {
    NMEAGPS_INVALIDATE( status );
    if (chr == 'N')
      m_fix.status = gps_fix::STATUS_NONE;
    else if (chr == 'T')
      m_fix.status = gps_fix::STATUS_TIME_ONLY;
    else if (chr == 'R')
      m_fix.status = gps_fix::STATUS_EST;
    else if (chr == 'G')
      m_fix.status = gps_fix::STATUS_STD;
    else if (chr == 'D')
      m_fix.status = gps_fix::STATUS_DGPS;

  } else if (chrCount == 1) {

    if (((chr == 'T') && (m_fix.status == gps_fix::STATUS_TIME_ONLY)) ||
        ((chr == 'K') && (m_fix.status == gps_fix::STATUS_EST)) ||
        (((chr == '2') || (chr == '3')) &&
         ((m_fix.status == gps_fix::STATUS_STD) ||
          (m_fix.status == gps_fix::STATUS_DGPS))) ||
        ((chr == 'F') && (m_fix.status == gps_fix::STATUS_NONE)))
      // status based on first char was ok guess
      m_fix.valid.status = true;

    else if ((chr == 'R') && (m_fix.status == gps_fix::STATUS_DGPS)) {
      m_fix.status = gps_fix::STATUS_EST; // oops, was DR instead
      m_fix.valid.status = true;
    }
  }
  
  return true;
}

bool emcNMEA::parse360( whole_frac & angle, char chr) //0<=angle<360
{
	bool done = parseFloat( angle, chr, 2 );
    if (done) {
    	//aunque ya tengamos el float con sus 2 decimales
    	//no termina este campo hasta que llegue una ,
    	//descartando todos los decimales entre 2 y max_char
    	if (chr!=',') done = false;

    	if (negative || (angle.whole >= 360)) {
    	  angle.whole =0;
    	  sentenceInvalid();
    	}
    }

    return done;
}

bool emcNMEA::parse180( whole_frac & angle, char chr, bool & field_informed  ) //-180<=angle<180
{
	bool done = parseFloat( angle, chr, 2 );
    if (done) {
    	  if ((angle.whole >= 180) || (angle.whole<-180)) {
    		  angle.whole =0;
    		  sentenceInvalid();
    	  } else if (chrCount > 0) field_informed = true;  // End of field, make sure it is informed
    }

  return done;
}

bool emcNMEA::parseXTEUnits( char chr )
{
    if (chrCount == 0) {

        // First char can only be 'K' or 'N'
        if (chr == 'K'||chr == 'N' ) {

    		if (chr == 'K') { // If K convert to N as distance always stored in Nautical miles
    			INorder.APB.XTE.Towf_00(INorder.APB.XTE.float_00() * float(0.54));//K to NM = 0.54;
    		}

        } else {
          sentenceInvalid();
        }

        // Second char can only be ','
      } else if ((chrCount > 1) || (chr != ',')) {
        sentenceInvalid();
    }

  return true;
} // parseXTEUnits


bool emcNMEA::parseDirSteer( char chr )
{

    if (chrCount == 0) {

        // First char can only be 'R' or 'L'
        if (chr == 'R'||chr == 'L' ) {

        	INorder.APB.dirSteer = chr;

        } else {
          sentenceInvalid();
        }

        // Second char can only be ','
      } else if ((chrCount > 1) || (chr != ',')) {
        sentenceInvalid();
    }

  return true;
} // parsedirSteer


bool emcNMEA::parseWindDir( char chr )
{

    if (chrCount == 0) {

        // First char can only be 'R' or 'L'
        if (chr == 'R'||chr == 'L' ) {

        	INorder.VWR.windDirLR = chr;

        } else {
          sentenceInvalid();
        }

        // Second char can only be ','
      } else if ((chrCount > 1) || (chr != ',')) {
        sentenceInvalid();
    }

  return true;
} // parseWindDir

bool emcNMEA::parseAngleRef( whole_frac & angle, char chr)
{

    if (chrCount == 0) {

        // First char can only be 'M'agnetic or 'T'
    	//Compatibility with AvNav also '' instead of 'T'
        if (chr == 'M'||chr == 'T'||chr == ',') {

    		if (chr == 'T'||chr == ',') { // If T convert to M as angles are always stored in Magnetic.
    			angle.Towf_00(getMagnetic(angle.float_00()));

    		}
        }

        // Second char can only be ','
      } else if ((chrCount > 1) || (chr != ',')) {
        sentenceInvalid();
    }

  return true;

} // parseAngleRef



bool emcNMEA::parseSide( char chr,  bool & field_informed )
{

    if (chrCount == 0) {

        // First char can only be 'S'tarboard or 'P'ortboard or empty ','
    	switch (chr) {
    	case 'S':
    		INorder.instParam.instSide = STARBOARD;
    		field_informed = true;
    		break;
    	case 'P':
    		INorder.instParam.instSide = PORTBOARD;
    		field_informed = true;
    		break;
    	case ',':
    		field_informed = false;
    		break;
    	default:
    		sentenceInvalid();
    		break;
        }

        // Second char can only be ','
      } else if ((chrCount > 1) || (chr != ',')) {
        sentenceInvalid();
    }

  return true;

} // parseSide

bool emcNMEA::parseDeadbandType( char chr )
{

    if (chrCount == 0) {

        // First char can only be 'm' or 'M' or 'A'
    	switch (chr) {
    	case 'M':
    		INorder.PIDgain.DBConfig = MAXDB;
    		break;
    	case 'm':
    		INorder.PIDgain.DBConfig = MINDB;
    		break;
    	case 'A':
    		INorder.PIDgain.DBConfig = AUTODB;
    		break;
    	default:
          sentenceInvalid();
        }

        // Second char can only be ','
      } else if ((chrCount > 1) || (chr != ',')) {
        sentenceInvalid();
    }

  return true;

} // parseDeadbandType

bool emcNMEA::parseAPmode( char chr )
{

    if (chrCount == 0) {

        // First char can only be 'S' or 'A' or 'T'
		switch (chr) {
		case 'S':
			INorder.APinfo.mode = STAND_BY;
			break;
		case 'A':
			INorder.APinfo.mode = AUTO_MODE;
			break;
		case 'T':
			INorder.APinfo.mode = TRACK_MODE;
			break;
		default:
			sentenceInvalid();
			break;
        }

        // Second char can only be ','
      } else if ((chrCount > 1) || (chr != ',')) {
        sentenceInvalid();
    }

  return true;

} // parseAPmode

bool emcNMEA::parseAlarmCircle( char chr ){

    if (chrCount == 0) {

    	//default value 'V'
        // First char can only be 'A' or 'V'
        if (chr == 'A'||chr == 'V'||chr == ',') {
        	INorder.APB.alarmCircle = chr;
        	if (INorder.APB.alarmCircle == ',') INorder.APB.alarmCircle = 'V';

        } else {
          sentenceInvalid();
        }

        // Second char can only be ','
      } else if ((chrCount > 1) || (chr != ',')) {
        sentenceInvalid();
    }
return true;
}

bool emcNMEA::parseAlarmPerp( char chr ){

    if (chrCount == 0) {

    	//default value 'V'
        // First char can only be 'A' or 'V'
        if (chr == 'A'||chr == 'V'||chr == ',') {
        	INorder.APB.alarmPerp = chr;
        	if (INorder.APB.alarmPerp == ',') INorder.APB.alarmPerp = 'V';

        } else {
          sentenceInvalid();
        }

        // Second char can only be ','
      } else if ((chrCount > 1) || (chr != ',')) {
        sentenceInvalid();
    }
return true;

}

bool emcNMEA::parseWPID( char * WPID, char chr){  //WP ID: Up to 4 characters, rest ignored.
	bool done = false;

	if (chrCount == 0) comma_needed(true);

	if (chr == ',') {
		// End of field, make sure it's finished with end of string (\0)
		*(WPID + chrCount)='\0';
	    done = true;
	    } else if (chrCount<4) {
	    	//only takes first 4 WPID characters, rest are ignored
	    	*(WPID + chrCount)=chr;
	    }

	return done;
}

bool emcNMEA::parseIntAuto( int8_t &val, uint8_t chr, bool & field_informed) {
	bool done = parseInt (val, chr);

    // End of field, make sure it is informed
    if (done and (chrCount > 0)) field_informed = true;

	return done;
}

bool emcNMEA::parseIntAuto( uint16_t &val, uint8_t chr, bool & field_informed) {
	bool done = parseInt (val, chr);

    // End of field, make sure it is informed
    if (done and (chrCount > 0)) field_informed = true;

	return done;
}

bool emcNMEA::parseIntAuto( uint8_t &val, uint8_t chr, bool & field_informed) {
	bool done = parseInt (val, chr);

    // End of field, make sure it is informed
    if (done and (chrCount > 0)) field_informed = true;

	return done;
}

bool emcNMEA::parseIntAuto( int16_t &val, uint8_t chr, bool & field_informed) {
	bool done = parseInt (val, chr);

    // End of field, make sure it is informed
    if (done and (chrCount > 0)) field_informed = true;

	return done;
}


bool emcNMEA::parseFloat ( whole_frac & val, char chr, uint8_t max_decimal, bool & field_informed ) {
	bool done = parseFloat ( val,  chr,  max_decimal);

    // End of field, make sure it is informed
    if (done and (chrCount > 0)) field_informed = true;

	return done;
}
bool emcNMEA::parseFloat ( whole_frac & val, char chr, uint8_t max_decimal) {
  bool done = false;

  if (chrCount == 0) {
    val.init();
    comma_needed( true );
    decimal      = 0;
    negative     = (chr == '-');
    if (negative) return done;
  }

  if (chr == ',') {
    // End of field, make sure it's scaled up
    if (!decimal)
      decimal = 1;
    if (val.frac)
      while (decimal++ <= max_decimal)
        val.frac *= 10;
    if (negative) {
      val.frac  = -val.frac;
      val.whole = -val.whole;
    }
    done = true;
  } else if (chr == '.') {
    decimal = 1;
  } else if (validateChars() && !isdigit(chr)) {
    sentenceInvalid();
  } else if (!decimal) {
    val.whole = val.whole*10 + (chr - '0');
  } else if (decimal++ <= max_decimal) {
    val.frac = val.frac*10 + (chr - '0');
  }

  return done;

} // parseFloat

bool emcNMEA::parseFloat ( whole_frac32 & val, char chr, uint8_t max_decimal, bool & field_informed ) {
	bool done = parseFloat ( val,  chr,  max_decimal);

    // End of field, make sure it is informed
    if (done and (chrCount > 0)) field_informed = true;

	return done;
}
bool emcNMEA::parseFloat ( whole_frac32 & val, char chr, uint8_t max_decimal) {
  bool done = false;

  if (chrCount == 0) {
    val.init();
    comma_needed( true );
    decimal      = 0;
    negative     = (chr == '-');
    if (negative) return done;
  }

  if (chr == ',') {
    // End of field, make sure it's scaled up
    if (!decimal)
      decimal = 1;
    if (val.frac)
      while (decimal++ <= max_decimal)
        val.frac *= 10;
    if (negative) {
      val.frac  = -val.frac;
      val.whole = -val.whole;
    }
    done = true;
  } else if (chr == '.') {
    decimal = 1;
  } else if (validateChars() && !isdigit(chr)) {
    sentenceInvalid();
  } else if (!decimal) {
    val.whole = val.whole*10 + (chr - '0');
  } else if (decimal++ <= max_decimal) {
    val.frac = val.frac*10 + (chr - '0');
  }

  return done;

} // parseFloat


// ---------------------------------------------

void emcNMEA::printPEMC_00(Stream * outStream){
	bufferStream->print ("$PEMC,00");
	send( outStream, string2char(output) );
	output.remove(0);
}

void emcNMEA::printPEMC_01(char changeRate, Stream * outStream) {

	printPEMC_0102(1, changeRate, outStream);
}

void emcNMEA::printPEMC_02(char changeRate, Stream * outStream) {

	printPEMC_0102(2, changeRate, outStream);
}

void emcNMEA::printPEMC_0102(uint8_t num, char changeRate, Stream * outStream){

	switch (changeRate){
	case 'i':
	case 'I':
	case 'r':
	case 'R':
	case 'P':
	case 'S':
		break;
	default:
		return;
	}

	switch (num){
	case 1:
	case 2:
		break;
	default:
		return;
	}

	bufferStream->print ("$PEMC,0");
	bufferStream->print (num);
	bufferStream->print (",");
	bufferStream->print (changeRate);

	send( outStream, string2char(output) );
	output.remove(0);
}

void emcNMEA::printPEMC_03(Stream * outStream) {
	printPEMC_0304(3,outStream);
    }

void emcNMEA::printPEMC_04(Stream * outStream) {
	printPEMC_0304(4,outStream);
    }

void emcNMEA::printPEMC_0304(uint8_t num, Stream * outStream) {
	switch (num){
	case 3:
	case 4:
		break;
	default:
		return;
	}
	if (OUTorder.instParam.isValid) {
		bufferStream->print ("$PEMC,0");
		bufferStream->print (num);
		bufferStream->print (",");
		bufferStream->print(OUTorder.instParam.centerTiller);
		bufferStream->print(',');
		bufferStream->print(OUTorder.instParam.maxRudder);
		bufferStream->print(',');
		bufferStream->print(OUTorder.instParam.avgSpeed);
		bufferStream->print(',');
    	switch (OUTorder.instParam.instSide) {
    	case STARBOARD:
    		bufferStream->print("S");
    		break;
    	case PORTBOARD:
    		bufferStream->print("P");
    		break;
        }
		bufferStream->print(',');
		bufferStream->print(OUTorder.instParam.rudDamping);
		bufferStream->print(',');
		bufferStream->print(OUTorder.instParam.magVariation.float_00());
		bufferStream->print(',');
		bufferStream->print(OUTorder.instParam.headAlign.float_00());
		bufferStream->print(',');
		bufferStream->print(OUTorder.instParam.offcourseAlarm);
		send( outStream, string2char(output) );
		output.remove(0);
    }
}

void emcNMEA::printPEMC_05(Stream * outStream) {
	printPEMC_0506(5,outStream);
    }

void emcNMEA::printPEMC_06(Stream * outStream) {
	printPEMC_0506(6,outStream);
    }


void emcNMEA::printPEMC_0506(uint8_t num, Stream * outStream) {
	switch (num){
	case 5:
	case 6:
		break;
	default:
		return;
	}
	if (OUTorder.PIDgain.isValid) {
		bufferStream->print ("$PEMC,0");
		bufferStream->print (num);
		bufferStream->print (",");
		bufferStream->print(OUTorder.PIDgain.gain.Kp.float_00());
		bufferStream->print(',');
		bufferStream->print(OUTorder.PIDgain.gain.Ki.float_00());
		bufferStream->print(',');
		bufferStream->print(OUTorder.PIDgain.gain.Kd.float_00());
		bufferStream->print(',');
		bufferStream->print(OUTorder.PIDgain.sTime);
		bufferStream->print(',');

    	switch (OUTorder.PIDgain.DBConfig) {
    	case MAXDB:
    		bufferStream->print("M");
    		break;
    	case MINDB:
    		bufferStream->print("m");
    		break;
    	case AUTODB:
    		bufferStream->print("A");
    		break;
        }

		send( outStream, string2char(output) );
		output.remove(0);
    }
}

void emcNMEA::printPEMC_07(Stream * outStream) {

	if (OUTorder.APinfo.isValid) {
		bufferStream->print ("$PEMC,07,");

    	switch (OUTorder.APinfo.mode) {
    	case AUTO_MODE:
    		bufferStream->print("A");
    		break;
    	case TRACK_MODE:
    		bufferStream->print("T");
    		break;
    	case STAND_BY:
    		bufferStream->print("S");
    		break;
//    	case CAL_IMU_MINIMUM:
    	case CAL_IMU_COMPLETE:
    	case CAL_FEEDBACK:
    		bufferStream->print("C");
    		break;
        }
		bufferStream->print(',');
		bufferStream->print(OUTorder.APinfo.rudder);
		bufferStream->print(',');
		bufferStream->print(OUTorder.APinfo.HDM.float_00());
		bufferStream->print(',');
		bufferStream->print(OUTorder.APinfo.CTS.float_00());
		bufferStream->print(',');
		// TODO: Update I/F Documentation: deadband is float
		bufferStream->print(OUTorder.APinfo.deadband.float_00());
		//Obsolete trim but respects message format
		bufferStream->print(",0.00");

		send( outStream, string2char(output) );
		output.remove(0);
    }
}

void emcNMEA::printPEMC_08(char RfI, Stream * outStream){
	switch (RfI){
	case 'I':
	case 'G':
	case 'A':
		break;
	default:
		return;
	}

	bufferStream->print ("$PEMC,08,");
	bufferStream->print (RfI);

	send( outStream, string2char(output) );
	output.remove(0);

};


void emcNMEA::printPEMC_12(Stream * outStream) {
	if (OUTorder.IMUcal.isValid) {
		bufferStream->print ("$PEMC,12,");
		bufferStream->print(OUTorder.IMUcal.IMUcalstatus);
		bufferStream->print(',');

			bufferStream->print( OUTorder.IMUcal.SYSstatus );
			bufferStream->print(',');
			bufferStream->print( OUTorder.IMUcal.GYROstatus );
			bufferStream->print(',');
			bufferStream->print( OUTorder.IMUcal.ACCELstatus );
			bufferStream->print(',');
			bufferStream->print( OUTorder.IMUcal.MAGNstatus );
			bufferStream->print(',');

		if (OUTorder.IMUcal.IMUcalstatus == 'C') {
			bufferStream->print(OUTorder.IMUcal.X);
			bufferStream->print(',');
			bufferStream->print(OUTorder.IMUcal.Y);
			bufferStream->print(',');
			bufferStream->print(OUTorder.IMUcal.Z);
		} else {
			bufferStream->print(",,");
		}

		send( outStream, string2char(output) );
		output.remove(0);
    }
}

void emcNMEA::printPEMC_13(Stream * outStream) {
	if (OUTorder.FBKcal.isValid) {
		bufferStream->print ("$PEMC,13,");
		bufferStream->print(OUTorder.FBKcal.cal_minFeedback);
		bufferStream->print(',');
		bufferStream->print(OUTorder.FBKcal.cal_maxFeedback);

		send( outStream, string2char(output) );
		output.remove(0);
    }
}


void emcNMEA::printAPB(Stream * outStream) {

	if (OUTorder.APB.isValid) {
		bufferStream->print(OUTorder.APB.XTE.float_00());
		bufferStream->print(',');
		bufferStream->print(OUTorder.APB.dirSteer);
		bufferStream->print(",N,");
		bufferStream->print(OUTorder.APB.alarmCircle);
		bufferStream->print(',');
		bufferStream->print(OUTorder.APB.alarmPerp);
		bufferStream->print(',');
		bufferStream->print(OUTorder.APB.BOD.float_00());
		bufferStream->print(",M,");
		bufferStream->print(OUTorder.APB.destID);
		bufferStream->print(',');
		bufferStream->print(OUTorder.APB.BTW.float_00());
		bufferStream->print(",M,");
		bufferStream->print(OUTorder.APB.CTS.float_00());
		bufferStream->print(",M");



		send( outStream, string2char(output) );
		output.remove(0);
    }
}

//TODO. Use printDm() function to complete HDG message
void emcNMEA::printHDG(Stream * outStream) {
	bufferStream->print ("$APHDG,");
	bufferStream->print (getHeading(),1);// SPM TODO: Hay que mandar True NO Magnetic
	bufferStream->print (",");
	printDm();
	send( outStream, string2char(output) );
	output.remove(0);
}

// obsolete
void emcNMEA::printHDM(Stream * outStream) {
	bufferStream->print ("$APHDM,");
	bufferStream->print (getHeading(),1);
	bufferStream->print (",M");
	send( outStream, string2char(output) );
	output.remove(0);
}
// obsolete
void emcNMEA::printHDT(Stream * outStream) {
	bufferStream->print ("$APHDT,");
	bufferStream->print (getTrue(getHeading()),1);
	bufferStream->print (",T");
	send( outStream, string2char(output) );
	output.remove(0);
}

void emcNMEA::printRSA(Stream * outStream) {
	bufferStream->print ("$APRSA,");
	bufferStream->print ((float)getRudder()/10);
	bufferStream->print (",A,,");

	send( outStream, string2char(output) );
	output.remove(0);
}

void emcNMEA::printMemory(Stream * outStream) {
	bufferStream->print ("!Memory: ");
	bufferStream->print (freeMemory(), 1);
	bufferStream->print (" KB");
	send( outStream, string2char(output) );
	output.remove(0);
}

void emcNMEA::printDm() {
		//	Compass Variation and Deviation
		//	Note that magnetic compass are also subject to their own errors due to magnetic interferences of metals around; this is called DEVIATION (ES: Desv�o).
		//	For an state of the art 6DOF IMU, this error is detected and corrected on the fly. For instance, only acelerometer is used when magnetometer is not reliable.
		//	The assumption of this specification is IMU Deviation is 0.

		float dm=getDm();
		bufferStream->print(",");
		bufferStream->print(",");
		bufferStream->print(abs(dm),1);
		bufferStream->print(",");
		bufferStream->print((dm)>0?("E"):("W"));
}

char* emcNMEA::string2char(String command){
    if(command.length()!=0){
        char *p = const_cast<char*>(command.c_str());
        return p;
    }
    return NULL;
}


void emcNMEA::TXReset() {
	_TX=true;
	_DelayTXStart = millis();
	_TXorder = 0;
}

void emcNMEA::TXNext() {
	_TXorder++;
}

bool emcNMEA::IsTXtime () {
	// returns false if timer is ON and still RUNNING
	// returns true if timer is OFF or is ON but arrived to the limit TIME
	if ( !_TX or ( (millis() -_DelayTXStart) < DELAY_TX_TIME) ) {
		return false;}
	return true;
}

void emcNMEA::TX1Reset() {
	_TX1=true;
	_DelayTX1Start = millis();
	_TX1order = 0;
}

void emcNMEA::TX1Next() {
	_TX1order++;
}


bool emcNMEA::IsTX1time () {
	// returns false if timer is ON and still RUNNING
	// returns true if timer is OFF or is ON but arrived to the limit TIME
	if ( !_TX1 or ( (millis() -_DelayTX1Start) < DELAY_TX1_TIME) ) {
		return false;}
	return true;
}
