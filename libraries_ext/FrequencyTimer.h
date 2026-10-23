#ifndef FREQUENCY_TIMER_H
#define FREQUENCY_TIMER_H

#include <Arduino.h>
#include <stdint.h>

enum class TimerRegister : uint16_t {
	TR_WINDFFinput,
	TR_WINDFFresult,
	TR_WINDFFavg,
	TR_saturated,
    COUNT
};


bool hasFrequencyElapsed(
    TimerRegister timerRegister,
    unsigned long frequencyMs
);

#endif  // FREQUENCY_TIMER_H
