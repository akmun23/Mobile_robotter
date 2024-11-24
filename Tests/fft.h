#pragma once

#include <complex>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <iterator>
#include <algorithm>
#include <iostream>
#include <chrono>

// Define the value of pi
const double PI = 3.1415926536;

// FFTProcessing class declaration
class FFTProcessing {
public:
    // Constructor to initialize the class with required parameters
    FFTProcessing(int minMagnitude, const std::vector<int>& dtmfRowFrequencies, const std::vector<int>& dtmfColumnFrequencies, int frequencyTolerance);

    // Function to process the file and detect DTMF tones
    std::vector<double> processFile(std::ifstream &file, int sampleRate, int bufferSize);

private:
    int minMagnitude; // Minimum magnitude to consider a frequency as detected
    const std::vector<int>& dtmfRowFrequencies; // List of DTMF row frequencies
    const std::vector<int>& dtmfColumnFrequencies; // List of DTMF column frequencies
    int frequencyTolerance; // Allowed frequency tolerance for detection
    std::vector<char> receivedSignal; // Vector to store detected DTMF tones
    std::chrono::high_resolution_clock::time_point clockStartMessage; // Timer for message start
    std::chrono::high_resolution_clock::time_point clockStartTone; // Timer for tone start
    double timeToReadTone; // Time taken to read and calculate a tone
    double timeToSendMessage; // Time taken to send a message
    bool letterReceived; // Flag to indicate if a letter is received
    bool startOfMessageReceived; // Flag to indicate the start of a message

    // Helper function to reverse bits
    unsigned int bitReverse(unsigned int x, int log2n);

    // Function to read DTMF data from file in chunks
    std::vector<double> readDTMFDataFFT(std::ifstream &file, int sampleRate);
    // Function to perform FFT
    std::vector<std::complex<double>> fft(const std::vector<std::complex<double>>& input, int log2n);

    // Function to find dominant frequencies in the FFT result
    std::vector<double> findDominantFrequencies(const std::vector<std::complex<double>>& fftResult, int sampleRate);

    // Function to save the detected DTMF signal and manage message state
    bool saveSignal(std::vector<double> rowMags, std::vector<double> columnMags, int maxRow, int maxColumn);

    // Function to calculate the elapsed time
    double timePassed(std::chrono::high_resolution_clock::time_point start);

    // Function to display the received DTMF signal
    void displayReceivedSignal();



    //Function to get Correct DTMF Char
    char getDTMFCharacter(double rowFreq, double colFreq);

    double TimePassedFFT(std::chrono::high_resolution_clock::time_point start);

    //Function to get DTMF Message
    std::pair<int, std::string> ToneAndMessageHandling(char detectedTone, std::string Message);

    std::vector<double> checkOutputFile(std::string filename, double calculationTime);


};
