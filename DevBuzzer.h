/*
 * DevBuzzer.h
 *
 *  Created on: 16 abr. 2026
 *      Author: Sergio
 */

#ifndef DEVBUZZER_H_
#define DEVBUZZER_H_

#include <Arduino.h>

struct Note {
    int freq;      // Frequency in Hz (0 = silence)
    int duration;  // Duration in ms
};

struct Melody {
    const Note* notes;
    int length;
    bool repeat;
    uint8_t priority;   // 1 = low, 2 = medium, 3 = high
};

class DevBuzzer {
public:
	DevBuzzer();
	virtual ~DevBuzzer();
	void startMelody(const Melody& melody);
	bool updateMelody();
	void stopMelody();
	void setUp(int pin = A12);
	void snooze(bool snooze = true);

    static const Melody START;
    static const Melody ALARM;
    static const Melody ALARM_OFF;
    static const Melody WARNING;
    static const Melody INFO;
    static const Melody BEEP;
    static const Melody GOT;

private:
	// Player state
    const Melody* _currentMelody = nullptr;
    int _currentNote = 0;
    unsigned long _noteStartTime = 0;
    bool _playing = false;
    int _lastFreq = -1;
    int _pin = 1;
    bool _snooze = false;

    //static const Note _melody[];
    //static const int _melodyLength;
    static const Note melodyStart[];
    static const Note melodyAlarm[];
    static const Note melodyAlarmOff[];
    static const Note melodyWarning[];
    static const Note melodyInfo[];
    static const Note melodyBeep[];
    //static const Note melodyGOT[];
};

#endif /* DEVBUZZER_H_ */
