#ifndef AUDIO_H
#define AUDIO_H

#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <cmath>

#include <portaudio.h>
#include <fftw3.h>     // FFTW:      Provides a discrete FFT algorithm to get
#include <vector>

// For debuq file
#include <iostream>  // Include the input/output stream library
#include <fstream>   // Include the file stream library


#define SAMPLE_RATE 44100.0             // How many audio samples to capture every second (44100 Hz is standard)
#define FRAMES_PER_BUFFER 2205.0        // How many audio samples to send to our callback function for each channel
#define NUM_CHANNELS 2                  // Number of audio channels to capture

#define RecordTimeMs 10000              // How long to record audio for (ms)


// Define our callback data (data that is passed to every callback function call)
typedef struct {
    double* in;         // Input buffer, will contain our audio sample
} streamCallbackData;

// Callback data, persisted between calls. Allows us to access the data it
// contains from within the callback function.
static streamCallbackData* spectroData;


class Audio
{
private:

    PaError err;
    double sampleRatio = FRAMES_PER_BUFFER / SAMPLE_RATE;
    PaStreamParameters inputParameters;
    PaStream* stream;

public:
    Audio();
    static void checkErr(PaError err);
    static inline float min(float a, float b);

    void start();
    void end();

    void getDevices();

    // Testing functions
    static int streamCallbackTest(
        const void* inputBuffer,
        void* outputBuffer,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo* timeInfo,
        PaStreamCallbackFlags statusFlags,
        void* userData
        );
    void printData();
    static void analyseGoertzelOutput(std::vector<double> mags);

};

#endif // AUDIO_H
