#ifndef GOERTZEL_H
#define GOERTZEL_H

#include <vector>
#include <complex>
#include <string>
#include <chrono>
#include "magnitudeanalysis.h"

// Goertzel algorithm functions

// Audio processing class
class GoertzelTesting : public MagnitudeAnalysis{
public:
    // Constructor
    GoertzelTesting(double minMagnitude, double timeToReadTone, int signalSize);

    std::chrono::high_resolution_clock::time_point _startToneCalculation;

    std::vector<double> _timeAtMaxCorrect;

    int _signalSize = 0;

    double _calcTimeMax = 0;
    double _calcTimeMin = 0;
    double _timeSumToneCalculation = 0;
    double _timeSum = 0;

    int _NUM_ChannelsCompareProgram = 1;
    int _toneCounter = 0;
    int _maxCorrect = 0;
    int _correct = 0;
    int _incorrect = 0;
    int _messageCounter = 1;


    // TESTING
    int _MagnitudeCounter = 0;
    double _rowMagnitudeSum = 0;
    double _rowMaxMagnitude = 0;
    double _rowMinMagnitude = 0;
    double _colMagnitudeSum = 0;
    double _colMaxMagnitude = 0;
    double _colMinMagnitude = 0;


    // Public methods
    std::vector<double> processFile(std::ifstream &file, int sampleRate, int bufferSize, int NumberOfChannels);

    void goertzel(const std::vector<double>& samples, int targetFreq, int sampleRate, double& power);

    void analyseGoertzelOutput(const std::vector<double>& mags);


private:
    // Method to read DTMF data from file in chunks
    std::vector<double> readDTMFDataChunk(std::ifstream& inFile, int bufferSize);

    // Method to use Goertzel algorithm on the data
    void analyzeDataWithGoertzel(const std::vector<double>& data, const int &sampleRate);

    void calculateInputWithHammingWindow(std::vector<double> in);


};
#endif // GOERTZEL_H
