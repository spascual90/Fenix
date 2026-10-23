// USAGE EXAMPLE
/*
if (hasFrequencyElapsed(
	TimerRegister::TR_WINDFF,
	1000)) {
	DEBUG_sprintf("windFF", float(windFF));
}
*/


#include "FrequencyTimer.h"

namespace {

constexpr uint8_t MAX_TIMER_REGISTERS =
    static_cast<uint8_t>(TimerRegister::COUNT);

struct TimerEntry {
    uint16_t id;
    unsigned long lastTime;
    bool used;
};

TimerEntry timers[MAX_TIMER_REGISTERS] = {};

}  // namespace


bool hasFrequencyElapsed(
    TimerRegister timerRegister,
    unsigned long frequencyMs)
{
    const uint16_t registerId =
        static_cast<uint16_t>(timerRegister);

    const unsigned long now = millis();

    if (frequencyMs == 0) {
        return true;
    }

    int8_t freePosition = -1;

    for (uint8_t i = 0; i < MAX_TIMER_REGISTERS; ++i) {

        if (timers[i].used) {

            if (timers[i].id == registerId) {

                if ((now - timers[i].lastTime) >= frequencyMs) {
                    timers[i].lastTime = now;
                    return true;
                }

                return false;
            }

        } else if (freePosition < 0) {
            freePosition = static_cast<int8_t>(i);
        }
    }

    if (freePosition >= 0) {
        TimerEntry& timer = timers[freePosition];

        timer.id = registerId;
        timer.lastTime = now;
        timer.used = true;
    }

    return false;
}
