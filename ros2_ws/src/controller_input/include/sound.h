#ifndef SOUND_H
#define SOUND_H

#include <math.h>

namespace sound {
    double SineWave(double time, double freq, double amp) {
        double result;
        double tpc = 44100 / freq; // ticks per cycle
        double cycles = time / tpc;
        double rad = 2 * M_PI * cycles;
        double amplitude = 32767 * amp;
        result = amplitude * sin(rad);
        return result;
    }
}

#endif
