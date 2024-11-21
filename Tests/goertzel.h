#ifndef GOERTZEL_H
#define GOERTZEL_H

#include <vector>
#include <complex>
#include <string>
#include <chrono>

// Goertzel algorithm functions
void goertzel(const std::vector<double>& samples, int targetFreq, int sampleRate, double& power);
bool analyseGoertzelOutput(const std::vector<double>& mags);
bool SaveSignal(std::vector<double> rowMags, std::vector<double> columnMags, int maxRow, int maxColumn);
static double TimePassed(std::chrono::high_resolution_clock::time_point start);

// Audio processing class
class GoertzelTesting {
public:
    // Constructor
    GoertzelTesting();

    // Public methods
    void processFile(const std::string& filename, int sampleRate, int bufferSize);
    std::vector<double> processFileTest(const std::string& filename, int sampleRate, int bufferSize);


private:
    // Method to read DTMF data from file in chunks
    std::vector<double> readDTMFDataChunk(std::ifstream& inFile, int bufferSize);

    // Method to use Goertzel algorithm on the data
    void analyzeDataWithGoertzel(const std::vector<double>& data, int sampleRate);

    std::vector<double> checkOutputFile(std::string filename, double calculationTime);
};
#endif // GOERTZEL_H
