#ifndef AUDIO_H
#define AUDIO_H

#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <cmath>

#include <portaudio.h>
#include <vector>

// For debuq file
#include <iostream>  // Include the input/output stream library
#include <fstream>   // Include the file stream library


#define SAMPLE_RATE 44100.0             // How many audio samples to capture every second (44100 Hz is standard)
#define FRAMES_PER_BUFFER 1500.0        // How many audio samples to send to our callback function for each channel

#define NUM_CHANNELS 2                  // Number of audio channels to capture

#define RecordTimeMs 20000              // How long to record audio for (ms)


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

public:
    Audio();
    Audio(PaError err, PaStream* stream);

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
    *@brief Method to start the audio stream and record audio for a specified amount of time (RecordTimeMs)
    *
    *@param nothing
    *
    *@return nothing
    *
    */
    void start();



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
    *@brief Method to analyze the output from the Goertzel algorithm and calls ReactOnSignal to store the result
    *
    *@param std::vector<double> mags
    *
    *@return bool
    *
    */
    static bool analyseGoertzelOutput(std::vector<double> mags);


    /**
    *@brief Method to analyze the output from the Goertzel algorithm and print the results
    *
    *@param std::vector<double> rowMags, std::vector<double> columnMags, int maxRow, int maxColumn
    *
    *@return bool
    *
    */
    static bool SaveSignal(std::vector<double> rowMags, std::vector<double> columnMags, int maxRow, int maxColumn);


    /**
    *@brief Method to update the robots speed and direction according to the new message
    *
    *@param nothing
    *
    *@return nothing
    *
    */
    static void reactOnSignal();

};

#endif // AUDIO_H
