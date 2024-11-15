#include "ProgramRunner.h"


int main(){



    // Select which program to run
    /*
     * 1 - Normal program to just listen for DTMF tones through goertzel
     *
     * 2 - Jons program to compare DFT, FFT and goertzel
     *
     * 3 - Program to record and listen for DTMF tones used when recording files for testing
    */
    RunProgram(1);

    return 0;
}

