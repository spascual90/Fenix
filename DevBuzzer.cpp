/*
 * DevBuzzer.cpp
 *
 *  Created on: 16 abr. 2026
 *      Author: Sergio
 */

#include "DevBuzzer.h"

// --- RAW DATA ---

const Note DevBuzzer::melodyStart[] = {
    {500,100},{0,50},
    {700,100},{0,50},
    {900,150},{0,500}
};

const Note DevBuzzer::melodyAlarm[] = {
    {800, 100},   // inicio bajo
    {1200, 100},  // subida
    {1600, 200},  // llegada arriba
    {2000, 800},  // tono alto sostenido
    {0,   300}    // pausa antes de repetir (si repeat=true)
};

const Note DevBuzzer::melodyAlarmOff[] = {
    {1000, 50}, {0, 20},
    {1400, 60}, {0, 20},
    {2000, 80}
};

const Note DevBuzzer::melodyWarning[] = {
    {900,200},{0,200},
    {900,200},{0,400},

    {900,200},{0,200},
    {900,200},{0,400},

    {900,200},{0,200},
    {900,200},{0,800}
};

const Note DevBuzzer::melodyInfo[] = {
    {600,100},{0,50},
    {800,120}
};

const Note DevBuzzer::melodyBeep[] = {
    {1000, 150},
    {0,    100}
};

//const Note DevBuzzer::melodyGOT[] = {
//
//// --- BLOQUE 1 ---
//{392,250},{262,250},{311,125},{349,125},{392,250},{262,250},{311,125},{349,125},
//{392,250},{262,250},{311,125},{349,125},{392,250},{262,250},{311,125},{349,125},
//
//{392,250},{262,250},{330,125},{349,125},{392,250},{262,250},{330,125},{349,125},
//{392,250},{262,250},{330,125},{349,125},{392,250},{262,250},{330,125},{349,125},
//
//{392,750},{262,750},
//
//// --- BLOQUE 2 ---
//{311,125},{349,125},{392,500},{262,500},{311,125},{349,125},
//{294,2000},
//
//{349,750},{233,750},
//{311,125},{294,125},{349,500},{233,750},
//{311,125},{294,125},{262,2000},
//
//// --- REPETICIÓN ---
//{392,750},{262,750},
//
//{311,125},{349,125},{392,500},{262,500},{311,125},{349,125},
//{294,2000},
//
//{349,750},{233,750},
//{311,125},{294,125},{349,500},{233,750},
//{311,125},{294,125},{262,2000},
//
//// --- BLOQUE FINAL ---
//{392,750},{262,750},
//{311,125},{349,125},{392,500},{262,500},{311,125},{349,125},
//
//{294,1500},
//{349,750},{233,750},
//
//{294,375},{311,375},{294,375},{233,375},
//{262,2000},
//
//{523,1500},
//{466,1500},
//{262,1500},
//{392,1500},
//{311,1500},
//
//{311,750},{349,750},
//{392,2000},
//
//// --- FINAL ---
//{523,1500},
//{466,1500},
//{262,1500},
//{392,1500},
//{311,1500},
//
//{311,750},{294,750},
//
//{523,250},{392,250},{415,125},{466,125},{523,250},{392,250},{415,125},{466,125},
//{523,250},{392,250},{415,125},{466,125},{523,250},{392,250},{415,125},{466,125},
//
//{0,500},{831,125},{932,125},{1047,250},{784,250},{831,125},{932,125},
//{1047,250},{784,125},{831,125},{932,125},{1047,250},{784,250},{831,125},{932,125}
//
//};


// --- WRAPPERS ---

const Melody DevBuzzer::START = {
    DevBuzzer::melodyStart,
    sizeof(DevBuzzer::melodyStart) / sizeof(Note),
    false,
	1 // low priority
};

const Melody DevBuzzer::BEEP = {
    DevBuzzer::melodyBeep,
    sizeof(DevBuzzer::melodyBeep) / sizeof(Note),
	false,
	1 // low priority
};

const Melody DevBuzzer::ALARM = {
    DevBuzzer::melodyAlarm,
    sizeof(DevBuzzer::melodyAlarm) / sizeof(Note),
	true,
	3 // high priority
};

const Melody DevBuzzer::ALARM_OFF = {
    DevBuzzer::melodyAlarmOff,
    sizeof(DevBuzzer::melodyAlarmOff) / sizeof(Note),
    false,
    1 // low priority
};

const Melody DevBuzzer::WARNING = {
    DevBuzzer::melodyWarning,
    sizeof(DevBuzzer::melodyWarning) / sizeof(Note),
	false,
	2 // medium priority
};

const Melody DevBuzzer::INFO = {
    DevBuzzer::melodyInfo,
    sizeof(DevBuzzer::melodyInfo) / sizeof(Note),
	false,
	1 // low priority
};

//const Melody DevBuzzer::GOT = {
//    DevBuzzer::melodyGOT,
//    sizeof(DevBuzzer::melodyGOT) / sizeof(Note),
//	false,
//	1
//};



DevBuzzer::DevBuzzer(){
}

DevBuzzer::~DevBuzzer() {
	// TODO Auto-generated destructor stub
}

void DevBuzzer::setUp(int pin) {
	_pin = pin;
	//Setup Buzzer
	pinMode(_pin, OUTPUT); // Set buzzer - pin PIN_BUZZER as an output

}


void DevBuzzer::startMelody(const Melody& melody) {
	if (_snooze) return;

	// start only in case of different melody
    if (_playing && _currentMelody == &melody) {
        return;
    }

	// start only in case of higher priority
    if (_playing && _currentMelody != nullptr) {
        if (melody.priority < _currentMelody->priority) {
            return;
        }
    }

    _currentMelody = &melody;
    _currentNote = 0;
    _noteStartTime = millis();
    _playing = true;
    _lastFreq = -1;
}

// false: melody finished
// true: melody continues
bool DevBuzzer::updateMelody() {
    if ( !_playing || _snooze || _currentMelody == nullptr) return false;

    unsigned long now = millis();

    if (now - _noteStartTime >= _currentMelody->notes[_currentNote].duration) {
        _currentNote++;

        if (_currentNote >= _currentMelody->length) {

            if (_currentMelody->repeat) {
                // REINICIAR
                _currentNote = 0;
            } else {
                // PARAR
                stopMelody();
                return false;
            }
        }

        _noteStartTime = now;
        _lastFreq = -1;
    }

    int freq = _currentMelody->notes[_currentNote].freq;

    if (freq != _lastFreq) {
        if (freq > 0) {
            tone(_pin, freq);
        } else {
            noTone(_pin);
        }

        _lastFreq = freq;
    }
    return true;
}

void DevBuzzer::stopMelody() {
    noTone(_pin);
    _playing = false;
    _currentMelody = nullptr;
    _currentNote = 0;
    _lastFreq = -1;
}

void DevBuzzer::snooze(bool snooze) {
	_snooze = snooze;
	if (_snooze) stopMelody();
}
