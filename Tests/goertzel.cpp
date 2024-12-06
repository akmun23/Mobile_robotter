#include "goertzel.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>
#include <unistd.h>
#include <vector>
#include <algorithm>
std::vector<double> HammingWindow2;
std::vector<double> InputAfterHammingWindow2;


// Constructor
GoertzelTesting::GoertzelTesting(double minMagnitude, double timeToReadTone, int signalSize) : MagnitudeAnalysis(minMagnitude, timeToReadTone), _signalSize(signalSize){
    createHammingWindow(_signalSize);
    HammingWindow2 = getHammingWindow();
    InputAfterHammingWindow2.resize(_signalSize);
};


double pi = 3.14159265358979323846;

double delayBetweenCalculation = 200*1000; // 200 ms in microseconds

// Function to read DTMF data from file in chunks
std::vector<double> GoertzelTesting::readDTMFDataChunk(std::ifstream& inFile, int bufferSize) {
    std::vector<double> signal;
    signal.resize(bufferSize);
    double value;
    for (int i = 0; i < bufferSize; ++i) {

        if(inFile >> value){
            signal[i] = value;
        }else {
            signal.resize(i);
            break;
        }
    }
    return signal;
}

// Function to apply Goertzel algorithm
void GoertzelTesting::goertzel(const std::vector<double>& samples, int targetFreq, int sampleRate, double& power) {
    int numSamples = samples.size();
    double k0 = numSamples*targetFreq/sampleRate;

    double omega_I = cos(2*pi*k0/numSamples);
    //double omega_Q = sin(2*pi*k0/FRAMES_PER_BUFFER);
    double v = 0;
    double v1 = 0;
    double v2 = 0;

    for (int i = 0; i < numSamples; ++i) {
        v  = 2*omega_I*v1 - v2 + samples[i * _NUM_ChannelsCompareProgram];
        v2 = v1;
        v1 = v;
    }


    power = v1*v1 + v2*v2 - omega_I*v1*v2;
}

// Function to analyze the data using the GoertzelTesting algorithm
void GoertzelTesting::analyzeDataWithGoertzel(const std::vector<double>& data, const int& sampleRate) {
    std::vector<int> dtmfTones = { 697, 770, 852, 941, 1209, 1336, 1477, 1633 };
    std::vector<double> magnitudes(dtmfTones.size(), 0.0);
    _startToneCalculation = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < dtmfTones.size(); ++i) {
        goertzel(data, dtmfTones[i], sampleRate, magnitudes[i]);
    }
    double TimePassed = timePassed(_startToneCalculation);
    _timeSumToneCalculation += TimePassed;
    if (TimePassed > _calcTimeMax) {
        _calcTimeMax = TimePassed;
    }
    if (_calcTimeMin == 0 || TimePassed < _calcTimeMin) {
        _calcTimeMin = TimePassed;
    }
    usleep(delayBetweenCalculation);
    analyseMagnitudes(magnitudes);
}

void GoertzelTesting::calculateInputWithHammingWindow(std::vector<double> in) {

    for (int i = 0; i < _signalSize; ++i) {
        InputAfterHammingWindow2[i] = in[i] * HammingWindow2[i];
    }
}

// Function to process the file and detect DTMF tones in chunks
std::vector<double> GoertzelTesting::processFile(std::ifstream &file, int sampleRate, int bufferSize, int NumberOfChannels) {

    _NUM_ChannelsCompareProgram = NumberOfChannels;
    std::ofstream outputFileGoertzel;

    // Open the file and clear it so it is empty
    outputFileGoertzel.open("Goertzel_Test_Output.txt", std::ios_base::trunc);
    outputFileGoertzel.close();
    outputFileGoertzel.open("Goertzel_Test_Output.txt", std::ios_base::app); // The file is opened in append mode meaning that the data will be added to the end of the file
    outputFileGoertzel << "New sequence of messages" << std::endl;
    file.clear();
    file.seekg(0, std::ios::beg);
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

       std::vector<double> data;
       while (true) {
           data = readDTMFDataChunk(file, bufferSize);
           if (data.size() < bufferSize){
               break;
           }
           calculateInputWithHammingWindow(data);

           analyzeDataWithGoertzel(InputAfterHammingWindow2, sampleRate);

           _toneCounter++;
           checkMessageState(outputFileGoertzel, _correct, _incorrect, _messageCounter);

       }
    outputFileGoertzel.close();
    double elapsedTime = timePassed(start);
    /*
    std::cout << "Average row magnitude: " << rowMagnitudeSum/MagnitudeCounter << std::endl;
    std::cout << "Average column magnitude: " << colMagnitudeSum/MagnitudeCounter << std::endl;
    std::cout << "Max row magnitude: " << rowMaxMagnitude << std::endl;
    std::cout << "Min row magnitude: " << rowMinMagnitude << std::endl;
    std::cout << "Max column magnitude: " << colMaxMagnitude << std::endl;
    std::cout << "Min column magnitude: " << colMinMagnitude << std::endl;
    */

    double avgCalcTime = _timeSumToneCalculation/_toneCounter;
    return checkOutputFile("Goertzel_Test_Output.txt", elapsedTime, "Checked_Output.txt", avgCalcTime, _calcTimeMax, _calcTimeMin);

}

