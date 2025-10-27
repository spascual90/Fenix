#ifndef _EMCNMEA_H_
#define _EMCNMEA_H_

#include <Arduino.h>

#include "NMEAGPS_cfg.h"

#include <NMEAGPS.h>
#include "Order.h"
#include "Streamers.h"
#include "StringStream.h"

//NEW
#include "NeoGPS_cfg.h"
#include "GPSport.h"

// All configurations are managed in Fenix_config.h
#include "Fenix_config.h"
//# define DELAY_TX_TIME 1000 // period of NMEA transmission in millisecs (1000 = 1sec)
//# define DELAY_TX1_TIME 5000 // period of PEMC transmission in millisecs (5000 = 3sec)

//------------------------------------------------------------
// Enable/disable the parsing of specific proprietary NMEA sentences.
//
// Configuring out a sentence prevents its fields from being parsed.
// However, the sentence type may still be recognized by /decode/ and 
// stored in member /nmeaMessage/.  No valid flags would be available.

// Ublox proprietary messages do not have a message type.  These
// messages start with "$PEMC," which ends with the manufacturer ID.  The
// message type is actually specified by the first numeric field.  In order
// to parse these messages, /parse_mfr_ID/ must be overridden to set the
// /nmeaMessage/ to PEMC_00 during /parseCommand/.  When the first numeric
// field is completed by /parseField/, it may change /nmeamessage/ to one 
// of the other PEMC message types.

  #if !defined(NMEAGPS_PARSE_PROPRIETARY)
    #error NMEAGPS_PARSE_PROPRIETARY must be defined in NMEAGPS_cfg.h in order to parse PEMC messages!
  #endif

  #if !defined(NMEAGPS_PARSE_MFR_ID)
    #error NMEAGPS_PARSE_MFR_ID must be defined in NMEAGPS_cfg.h in order to parse PEMC messages!
  #endif


//=============================================================
// NMEA 0183 Parser for ublox Neo-6 GPS Modules.
//
// @section Limitations
// Very limited support for ublox proprietary NMEA messages.
// Only NMEA messages of types PEMC,00 and PEMC,04 are parsed.
//

class emcNMEA : public NMEAGPS
{
    //emcNMEA( const emcNMEA & );

public:

    emcNMEA () ;

    //Class I/F
    virtual float getDm(void) = 0;
    virtual float getMagnetic(float value) = 0;
    virtual float getTrue(float value) = 0;
    virtual int getRudder(void) = 0;
    virtual float getHeading(void) = 0;

    bool parseIntAuto( uint16_t &val, uint8_t chr, bool & field_informed);
    bool parseIntAuto( int8_t &val, uint8_t chr, bool & field_informed);
    bool parseIntAuto( int16_t &val, uint8_t chr, bool & field_informed);
    bool parseIntAuto( uint8_t &val, uint8_t chr, bool & field_informed);

	int getTxorder() const {
		return _TXorder;
	}
	int getTx1order() const {
		return _TX1order;
	}

protected:

    void sentenceOk() ;
    void sentenceInvalid() ;
    void sentenceUnrecognized() ;
    NMEAGPS_VIRTUAL decode_t decode( char c ) override;
    NMEAGPS_VIRTUAL bool parseAPB( char chr );
    NMEAGPS_VIRTUAL bool parseHDM( char chr );
    NMEAGPS_VIRTUAL bool parseVWR( char chr );
    NMEAGPS_VIRTUAL bool parseRMC( char chr );
    NMEAGPS_VIRTUAL bool parseVTG( char chr );
    bool parse360(whole_frac & angle, char chr);
    bool parse180(whole_frac & angle, char chr );
    bool parse180( whole_frac & angle, char chr, bool & field_informed);
    bool parseAngleRef( whole_frac & angle, char chr);
    bool parseSide( char chr,  bool & field_informed );
    bool parseDeadbandType( char chr );
    bool parseAPmode( char chr );
    bool parseDirSteer( char chr );
    bool parseWindDir( char chr );
    bool parseXTEUnits( char chr );
    bool parseAlarmCircle( char chr );// AlarmCircle: A - Activated, V - no alarm.
    bool parseAlarmPerp( char chr );// AlarmPerp: A - Activated, V - no alarm.
    bool parseWPID( char * WPID, char chr );//WP ID: Up to 4 characters, rest ignored.
    bool parseFloat ( whole_frac & val, char chr, uint8_t max_decimal );
    bool parseFloat ( whole_frac & val, char chr, uint8_t max_decimal, bool & field_informed );
    bool parseFloat ( whole_frac32 & val, char chr, uint8_t max_decimal );
    bool parseFloat ( whole_frac32 & val, char chr, uint8_t max_decimal, bool & field_informed );

    //Timer
    void TXReset(void);
    bool IsTXtime (void);
    void TXNext(void);
    //Timer#1
    void TX1Reset(void);
    bool IsTX1time (void);
    void TX1Next(void);

    enum EXT_msg_t {
    	PEMC_00 = NMEA_LAST_MSG+1, // SWITCH_MODE
        PEMC_01, // INC_RUDD1,INC_RUDD10,DEC_RUDD1,DEC_RUDD10
		PEMC_02, // INC_CTS1,INC_CTS10,INC_CTS100,DEC_CTS1,DEC_CTS10,DEC_CTS100
		PEMC_03, // GET_INST
		PEMC_04, // SET_INST
		PEMC_05, // GET_GAIN
		PEMC_06, // SET_GAIN
		PEMC_07, // GET_APINFO
		PEMC_08, //REQUEST
		PEMC_09, //CALIB_MODE
		PEMC_10, //EE_FBK_CALIB
		PEMC_11, //SAVE
		PEMC_12, //IMUCal
		PEMC_13, //FEEDBACK VALUES
		PEMC_14, //calibrate_py Calib. values

        EXT_END
    };
    static const nmea_msg_t EXT_FIRST_MSG = (nmea_msg_t) PEMC_00;
    static const nmea_msg_t EXT_LAST_MSG  = (nmea_msg_t) (EXT_END-1);

    void printPEMC_00(Stream * outStream);
    void printPEMC_01(char changeRate, Stream * outStream);
    void printPEMC_02(char changeRate, Stream * outStream);
    void printPEMC_03(Stream * outStream);
    void printPEMC_04(Stream * outStream);
    void printPEMC_05(Stream * outStream);
    void printPEMC_06(Stream * outStream);
    void printPEMC_07(Stream * outStream);
    void printPEMC_08(char RfI, Stream * outStream);
    void printPEMC_12(Stream * outStream);
    void printPEMC_13(Stream * outStream);

    void printAPB(Stream * outStream);
    void printHDG(Stream * outStream);
    void printHDM(Stream * outStream);
    void printHDT(Stream * outStream);
    void printRSA(Stream * outStream);
    void printMemory(Stream * outStream);

	void printDm();


	SERIALorder INorder; // Order parsed after reception
    SERIALorder OUTorder; // Order compiled before sending

    bool parseMfrID( char chr )
      { bool ok;

        switch (chrCount) {
          case  1: ok = (chr == 'E'); break;
          case  2: ok = (chr == 'M'); break;
          default: if (chr == 'C') {
                     ok = true;
                     nmeaMessage = (nmea_msg_t) PEMC_00;
                   } else
                     ok = false;
                   break;
        }
        return ok;
      };

    bool classifyPEMC( char chr );
    bool parsePEMC_01( char chr );
    bool parsePEMC_02( char chr );
    bool parsePEMC_0304( char chr );
    bool parsePEMC_0506( char chr );
    bool parsePEMC_07( char chr );
    bool parsePEMC_08( char chr );
    bool parsePEMC_09( char chr );
    bool parsePEMC_11( char chr );
    bool parsePEMC_14( char chr );

    bool parseField( char chr );
    bool parseFix( char chr );

    // COMPILER FUNCTIONS
    String output;
    StringStream * bufferStream;

    char* string2char(String command);

    void printPEMC_0102(uint8_t num, char changeRate, Stream * outStream);
    void printPEMC_0304(uint8_t num, Stream * outStream);
    void printPEMC_0506(uint8_t num, Stream * outStream);


private:
    //TIMER
	bool _TX=false;
	unsigned long _DelayTXStart = millis();
	//ORDER OF NMEA Transmition per loop
	int _TXorder = 0;

    //TIMER#1
	bool _TX1=false;
	unsigned long _DelayTX1Start = millis();
	//ORDER OF NMEA Transmition per loop
	int _TX1order = 0;

} ;


#endif
