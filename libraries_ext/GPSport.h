#ifndef GPSport_h
#define GPSport_h

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

//
// SPM INI
#define SERIAL_IF_AVAILABLE // Comment this line to avoid SERIAL IF compilation: NMEA and debugging will not be available
#include <PString.h> /// http://arduiniana.org/libraries/pstring/
#ifdef SERIAL_IF_AVAILABLE
// SPM FIN
             #include <NeoHWSerial.h>

             #define gpsPort NeoSerial
             #define GPS_PORT_NAME "NeoSerial"// (USB D0, D1)"

             #define DEBUG_PORT NeoSerial // Comment this line to avoid debugging (NMEA IF will continue available)
			 #ifdef DEBUG_PORT
			 #define DEBUG_PORT_NAME "NeoSerial"// (USB D0, D1)"
			 #endif

#endif
// SPM INI
// Define Bluetooth interface
// SPM  ATENCION  INI COMENTAR PARA DEBUG NANO - Weather Station!!!
	#define HMI_BT // Bluetooth IF
    #define BTPort NeoSerial1
    #define BT_PORT_NAME "NeoSerial1"// (D18, D19)
// SPM  ATENCION FIN COMENTAR PARA DEBUG NANO - Weather Station!!!

			//SPM ATENCION AÑADIR PARA DEBUG NANO
            //#define BTPort NeoSerial
            //#define BT_PORT_NAME "NeoSerial"


static char DEBUG_buffer[80];

inline void DEBUG_print (const char str[]=DEBUG_buffer){
	#ifdef DEBUG_PORT
	DEBUG_PORT.print(str);
	#endif
}

inline void DEBUG_print (const __FlashStringHelper *t)
{
	#ifdef DEBUG_PORT
	char buf[30] ;
	PString s(buf, 30) ;
	s.print(t) ;
	DEBUG_print(buf) ;
	#endif
}

inline void DEBUG_sprintf(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(DEBUG_buffer, sizeof(DEBUG_buffer), fmt, args);
    va_end(args);
    DEBUG_print();
}

inline void DEBUG_sprintfree(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vsnprintf(DEBUG_buffer, sizeof(DEBUG_buffer), fmt, args);
    va_end(args);
    DEBUG_print();
}

// ======================================================
//  DEBUG_sprintf : imprime valores int con etiqueta
// ======================================================
//DEBUG_sprintf("Error code", getErrorCode());
//DEBUG_sprintf("Rudder limits", limit_min, limit_center, limit_max);

// --- un valor ---
inline void DEBUG_sprintf(const char* label, int v1) {
    DEBUG_sprintf("%s: %i\n", label, v1);
}

// --- dos valores ---
inline void DEBUG_sprintf(const char* label, int v1, int v2) {
    DEBUG_sprintf("%s: %i, %i\n", label, v1, v2);
}

// --- tres valores ---
inline void DEBUG_sprintf(const char* label, int v1, int v2, int v3) {
    DEBUG_sprintf("%s: %i, %i, %i\n", label, v1, v2, v3);
}

// ======================================================
//  DEBUG_sprintf : imprime valores float con etiqueta
// ======================================================

// --- un valor ---
inline void DEBUG_sprintf(const char* label, float v1, int l = 6, int d = 2) {
    char c1[l + 3];
    dtostrf(v1, l, d, c1);
    DEBUG_sprintf("%s: %s\n", label, c1);
}

// --- dos valores ---
inline void DEBUG_sprintf(const char* label, float v1, float v2, int l = 6, int d = 2) {
    char c1[l + 3];
    char c2[l + 3];
    dtostrf(v1, l, d, c1);
    dtostrf(v2, l, d, c2);
    DEBUG_sprintf("%s: %s, %s\n", label, c1, c2);
}

// --- tres valores ---
inline void DEBUG_sprintf(const char* label, float v1, float v2, float v3, int l = 6, int d = 2) {
    char c1[l + 3];
    char c2[l + 3];
    char c3[l + 3];
    dtostrf(v1, l, d, c1);
    dtostrf(v2, l, d, c2);
    dtostrf(v3, l, d, c3);
    DEBUG_sprintf("%s: %s, %s, %s\n", label, c1, c2, c3);
}

static void DEBUG_flush (void){
	#ifdef DEBUG_PORT
	DEBUG_PORT.flush();
	#endif
}

#endif // SPM FIN

