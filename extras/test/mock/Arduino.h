// Minimal host-side stand-in for Arduino.h so easyPID can be compiled and
// exercised natively with deterministic, test-controlled time.
#pragma once

#include <stdint.h>
#include <stdlib.h>

// Mirrors the real Arduino.h definition exactly (double, no f suffix) so that
// any macro collision reproduces here just as it does on AVR/ESP32.
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// Virtual clock, advanced explicitly by the test so simulations are repeatable.
extern unsigned long g_mockMillis;
inline unsigned long millis() { return g_mockMillis; }
inline unsigned long micros() { return g_mockMillis * 1000UL; }
inline void mockAdvance(unsigned long ms) { g_mockMillis += ms; }
inline void mockSetMillis(unsigned long ms) { g_mockMillis = ms; }

inline long mockRandom(long howsmall, long howbig) {
    if (howsmall >= howbig) return howsmall;
    return howsmall + (rand() % (howbig - howsmall));
}
#define random mockRandom
