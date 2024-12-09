#ifndef DFT_H
#define DFT_H

#include "magnitudeanalysis.h"
#include <chrono>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <complex>
#include <string>

const double PI_M = 3.1415926536;



class DFT : public MagnitudeAnalysis{
private:
    // Constructor

    int _messageCounter = 1;
    int _correctMessages = 0;
    int _incorrectMessages = 0;
    int _timedOutMessages = 0;

    double _timeSum = 0;

    std::vector<double> _timeAtMaxCorrect;

    std::vector<int> _tones = {697, 770, 852, 941, 1209, 1336, 1477, 1633};


    double _TimeSumChunkDFT = 0;
    double _TimeSumCalculationDFT = 0;
    int _countDFT = 0;
    double _calcTimeMaxDFT = 0;
    double _calcTimeMinDFT = 0;




public:

    DFT(double minMagnitude, double timeToReadTone);

    DFT(double minMagnitude, double timeToReadTone, std::string outfile);

    std::vector<double> readDTMFDataChunk(std::ifstream& inFile, int& bufferSize);

    std::vector<double> runDFT(std::ifstream &file, int& sampleRate, int& bufferSize);

    // Function to compute the DFT
    void computeDFT(const std::vector<double>& input, int &sampleRate);

    std::vector<double> computeDFTCompare(float* input, int sampleRate, int FramesPerBuffer);

    // Function to read DTMF data from file
    std::vector<double> readDTMFDataDFT(std::ifstream &file, int& sampleRate);

};





#endif
