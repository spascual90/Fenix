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

#include "emcNMEA.h"


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
      case PEMC_11: return parsePEMC_11( chr ); //Save
      default: 
        break;
    }

  } else

    // Delegate
    return NMEAGPS::parseField(chr);


  return true;

} // parseField

// $--APB,A,A,x.x,a,N,A,A,x.x,a,c--c,x.x,a,x.x,a*hh<CR><LF>
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
	  case 13:parse360( INorder.APB.CTS, chr);  break; // Angle: Magnitude, Reference (M/T)
	  case 14: parseAngleRef(INorder.APB.CTS, chr);
		  INorder.APB.isValid=YES;
	  	  INorder.set_order(REQ_TRACK);
	  	  break;
	          }
	return ok;
}

// $--HDM,x.x,M*hh<CR><LF>
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

// $--VWR,x.x,a,,,,,,*hh<CR><LF>
//$IIVWR,045.0,L,12.6,N,6.5,M,23.3,K*52
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
							case '9': nmeaMessage = (nmea_msg_t) PEMC_09; INorder.set_order(START_CAL);	break;
							default : ok = false;
				}// end switch chr
			} else {// case true: // Message is PEMC,1#,...
				switch (chr) {
							// $PEMC,10
							case '0': nmeaMessage = (nmea_msg_t) PEMC_10; INorder.set_order(EE_FBK_CAL); break;
							// $PEMC,11
							case '1': nmeaMessage = (nmea_msg_t) PEMC_11; break;
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
    	case 6: parseInt( INorder.APinfo.deadband , chr ); break;
        case 7:
        	parseFloat( INorder.APinfo.trim , chr , 2, INorder.APinfo.flag.trim );
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


bool emcNMEA::parse360( whole_frac & angle, char chr ) //0<=angle<360
{
	bool done = parseFloat( angle, chr, 2 );
    if (done) {
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
        if (chr == 'M'||chr == 'T' ) {

    		if (chr == 'T') { // If T convert to M as angles are always stored in Magnetic.
    			angle.Towf_00(getMagnetic(angle.float_00()));

    		}

        } else {
          sentenceInvalid();
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

        // First char can only be 'A' or 'V'
        if (chr == 'A'||chr == 'V') {

        	INorder.APB.alarmCircle = chr;

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

        // First char can only be 'A' or 'V'
        if (chr == 'A'||chr == 'V') {

        	INorder.APB.alarmPerp = chr;

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
    	case CAL_IMU_MINIMUM:
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
		bufferStream->print(OUTorder.APinfo.deadband);
		bufferStream->print(',');
		bufferStream->print(OUTorder.APinfo.trim.float_00());

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

void emcNMEA::printHDG(Stream * outStream) {
	bufferStream->print ("$APHDG,");
	bufferStream->print (getHeading(),1);// SPM TODO: Hay que mandar True NO Magnetic
	bufferStream->print (",");
	printDm();
	send( outStream, string2char(output) );
	output.remove(0);
}

void emcNMEA::printHDM(Stream * outStream) {
	bufferStream->print ("$APHDM,");
	bufferStream->print (getHeading(),1);
	bufferStream->print (",M");
	send( outStream, string2char(output) );
	output.remove(0);
}

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

void emcNMEA::printDm() {
		//	Compass Variation and Deviation
		//	Note that magnetic compass are also subject to their own errors due to magnetic interferences of metals around; this is called DEVIATION (ES: Desvío).
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
}

bool emcNMEA::IsTX1time () {
	// returns false if timer is ON and still RUNNING
	// returns true if timer is OFF or is ON but arrived to the limit TIME
	if ( !_TX1 or ( (millis() -_DelayTX1Start) < DELAY_TX1_TIME) ) {
		return false;}
	return true;
}



