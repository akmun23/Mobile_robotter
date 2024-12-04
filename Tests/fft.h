#pragma once

#include <complex>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <iterator>
#include <algorithm>
#include <iostream>
#include <chrono>
#include "magnitudeanalysis.h"

// Define the value of pi
const double PI = 3.1415926536;

// FFTProcessing class declaration
class FFTProcessing : public MagnitudeAnalysis{
private:

    int _minMagnitude = 0;
    int _messageCounter = 0;
    int _frequencyTolerance = 0;

    double _TimeSumChunkFFT = 0;
    double _TimeSumCalculationFFT = 0;
    int _countFFT = 0;
    double _calcTimeMaxFFT = 0;
    double _calcTimeMinFFT = 0;

    double _calculationTime = 0;
    int _correctMessages = 0;
    int _incorrectMessages = 0;
    int _timedOutMessages = 0;
    std::vector<double> _data;

    std::vector<char> _receivedSignal; // Vector to store detected DTMF tones


    std::chrono::high_resolution_clock::time_point _TimeForEntireSequenceStartFFT;
    std::chrono::high_resolution_clock::time_point _TimeForCalculationStartFFT;


public:
    // Constructor to initialize the class with required parameters
    FFTProcessing(int minMagnitude, double timeToReadTone, int frequencyTolerance);

    // Function to process the file and detect DTMF tones
    std::vector<double> processFile(std::ifstream &file, int sampleRate, int bufferSize);

    // Function to perform FFT
    std::vector<std::complex<double>> fft(const std::vector<std::complex<double>>& input, int log2n);

    std::vector<double> readDTMFData(std::ifstream& inFile, int bufferSize);

    std::vector<double> findDominantFrequencies(const std::vector<std::complex<double>>& fftResult, int sampleRate);

    // Helper function to reverse bits
    unsigned int bitReverse(unsigned int x, int log2n);
};
