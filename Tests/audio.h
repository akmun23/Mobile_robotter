#ifndef AUDIO_H
#define AUDIO_H

#include "magnitudeanalysis.h"
#include <thread>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <cmath>
#include <portaudio.h>
#include <vector>
#include "fft.h"

// For debuq file
#include <iostream>  // Include the input/output stream library
#include <fstream>   // Include the file stream library


#define SAMPLE_RATE 16000.0             // How many audio samples to capture every second (44100 Hz is standard)
#define FRAMES_PER_BUFFER 400.0        // How many audio samples to send to our callback function for each channel

#define NUM_CHANNELS 1                  // Number of audio channels to capture (1 = mono, 2 = stereo) Set to 2 for goertzel and 1 for FFT

// Define our callback data (data that is passed to every callback function call)
typedef struct {
    double* in;         // Input buffer, will contain our audio sample
} streamCallbackData;

// Callback data, persisted between calls. Allows us to access the data it
// contains from within the callback function.
static streamCallbackData* spectroData;



class Goertzel : public MagnitudeAnalysis
{
private:

    PaError err;
    double sampleRatio = FRAMES_PER_BUFFER / SAMPLE_RATE;
    PaStreamParameters inputParameters;



public:
    Goertzel(double minMagnitude, double timeToReadTone);

    /**
    *@brief Method to check for errors in PortAudio functions
    *
    *@param PaError err
    *
    *@return nothing
    *
    */
    static void checkErr(PaError err);


    /**
    *@brief Method to return the minimum of two float values
    *
    *@param float a, float b
    *
    *@return float
    *
    */
    static inline float min(float a, float b);

    /**
    *@brief Method to initialize PortAudio and allocate memory for the input buffer
    *
    *@param nothing
    *
    *@return nothing
    *
    */
    void Init();


    /**
    *@brief Method to start the audio stream and record audio
    *
    *@param nothing
    *
    *@return nothing
    *
    */
    void start();

    /**
    *@brief Get the audio devices accessible to PortAudio and their specifications
    *
    *@param nothing
    *
    *@return nothing
    *
    */
    void getDevices();


    /**
    *@brief This function is called by PortAudio when audio data is available to be processed
    *
    *@param const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData
    *
    *@return nothing
    *
    */
    static int streamCallback(
        const void* inputBuffer,
        void* outputBuffer,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo* timeInfo,
        PaStreamCallbackFlags statusFlags,
        void* userData
    );


    static void calculateGoertzel(int tone, const float* in, std::vector<double>& mags, int magsIterator);

    /**
    *@brief Method to analyze the output from the GoertzelTesting algorithm and calls ReactOnSignal to store the result
    *
    *@param std::vector<double> mags
    *
    *@return bool
    *
    */
    static bool analyseGoertzelOutput(std::vector<double>& mags);


    /**
    *@brief Method to analyze the output from the GoertzelTesting algorithm and print the results
    *
    *@param std::vector<double> rowMags, std::vector<double> columnMags, int maxRow, int maxColumn
    *
    *@return bool
    *
    */
    static bool SaveSignal(std::vector<double> &rowMags, std::vector<double> &columnMags, int &maxRow, int &maxColumn);

    /**
    *@brief Method to end the audio stream and free allocated resources
    *
    *@param nothing
    *
    *@return nothing
    *
    */
    void end();

    /**
     * @brief Function to get the time passed since start
     *
     * @param std::chrono::high_resolution_clock::time_point start
     *
     * @return double
     */
    static double TimePassed(std::chrono::high_resolution_clock::time_point start);

    void InitForStoringInFile();

    void startTimedRecording(int RecordingTime);

    static int streamCallbackForStoringInFile(
        const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags,
        void* userData
    );

    /// FFT Test

    void InitForFFT();

    void startFFT();

    static int streamCallbackFFT(
        const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags,
        void* userData
    );


    void InitForCompare();

    void startCompare();

    static int streamCallbackCompare(
        const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags,
        void* userData
    );



};

#endif // AUDIO_H
