#pragma once
#include <vector>
#include <complex>
#include <string>

// Goertzel algorithm functions
void goertzel(const std::vector<double>& samples, int targetFreq, int sampleRate, double& power);
bool analyseGoertzelOutput(const std::vector<double>& mags);

// Audio processing class
class Audio {
public:
    // Constructor
    Audio();

    // Public methods
    void processFile(const std::string& filename, int sampleRate, int bufferSize);

private:
    // Method to read DTMF data from file in chunks
    std::vector<double> readDTMFDataChunk(std::ifstream& inFile, int bufferSize);

    // Method to use Goertzel algorithm on the data
    void analyzeDataWithGoertzel(const std::vector<double>& data, int sampleRate);
};



