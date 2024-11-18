#ifndef AUDIO_H
#define AUDIO_H

#include <thread>
#include <time.h>
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

#define NUM_CHANNELS 1                  // Number of audio channels to capture (1 = mono, 2 = stereo)

// Define our callback data (data that is passed to every callback function call)
typedef struct {
    double* in;         // Input buffer, will contain our audio sample
} streamCallbackData;

// Callback data, persisted between calls. Allows us to access the data it
// contains from within the callback function.
static streamCallbackData* spectroData;


class Goertzel
{
private:


    PaError err;
    double sampleRatio = FRAMES_PER_BUFFER / SAMPLE_RATE;
    PaStreamParameters inputParameters;


public:
    Goertzel();
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


    static void calculateGoertzel(int tone, const float* in, std::vector<double>& mags, int &magsIterator, double &omega_I, double &k0);

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
    *@brief Method to update the robots speed and direction according to the new message
    *
    *@param nothing
    *
    *@return nothing
    *
    */
    static void reactOnSignal();

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


    /**
     * @brief Same function as Init() but for storing the audio in a file
     *
     * @param nothing
     *
     * @return nothing
     */
    void InitForStoringInFile();


    /**
     * @brief Same as start() but only runs for a specified time
     *
     * @param int RecordingTime
     *
     * @return nothing
     */
    void startTimedRecording(int RecordingTime);


    /**
     * @brief Instead of reacting to the signal, store the audio in a file
     *
     * @param const void* inputBuffer, void* outputBuffer, unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo, PaStreamCallbackFlags statusFlags, void* userData
     *
     * @return nothing
     */
    static int streamCallbackForStoringInFile(
        const void* inputBuffer,
        void* outputBuffer,
        unsigned long framesPerBuffer,
        const PaStreamCallbackTimeInfo* timeInfo,
        PaStreamCallbackFlags statusFlags,
        void* userData
        );






};

#endif // AUDIO_H
