#include "audio.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>



// Constructor
Audio::Audio() {}


int pi = 3.14159265358979323846;

int MinMagnitude;

// Function to read DTMF data from file in chunks
std::vector<double> Audio::readDTMFDataChunk(std::ifstream& inFile, int bufferSize) {
    std::vector<double> signal;
    double value;
    for (int i = 0; i < bufferSize && inFile >> value; ++i) {
        signal.push_back(value);
    }
    return signal;
}

// Function to apply Goertzel algorithm
void goertzel(const std::vector<double>& samples, int targetFreq, int sampleRate, double& power) {
    int numSamples = samples.size();
    double scalingFactor = (2.0 * pi * targetFreq) / sampleRate;
    double cosine = cos(scalingFactor);
    double sine = sin(scalingFactor);
    double coeff = 2.0 * cosine;
    double q0 = 0.0;
    double q1 = 0.0;
    double q2 = 0.0;

    for (int i = 0; i < numSamples; ++i) {
        q0 = coeff * q1 - q2 + samples[i];
        q2 = q1;
        q1 = q0;
    }

    double real = q1 - q2 * cosine;
    double imag = q2 * sine;
    power = real * real + imag * imag;
}

// Function to analyze the data using the Goertzel algorithm
void Audio::analyzeDataWithGoertzel(const std::vector<double>& data, int sampleRate) {
    std::vector<int> dtmfTones = { 697, 770, 852, 941, 1209, 1336, 1477, 1633 };
    std::vector<double> magnitudes(dtmfTones.size(), 0.0);

    for (size_t i = 0; i < dtmfTones.size(); ++i) {
        goertzel(data, dtmfTones[i], sampleRate, magnitudes[i]);
    }

    if (analyseGoertzelOutput(magnitudes)) {
        std::cout << "DTMF tone detected!" << std::endl;
    }
    else {
        std::cout << "No DTMF tone detected." << std::endl;
    }
}

// Function to analyze Goertzel output and determine DTMF tones
bool analyseGoertzelOutput(const std::vector<double>& mags) {
    std::vector<double> rowMags(mags.begin(), mags.begin() + 4);
    std::vector<double> columnMags(mags.begin() + 4, mags.end());

    auto maxRowIter = std::max_element(rowMags.begin(), rowMags.end());
    auto maxColumnIter = std::max_element(columnMags.begin(), columnMags.end());

    int maxRow = std::distance(rowMags.begin(), maxRowIter);
    int maxColumn = std::distance(columnMags.begin(), maxColumnIter);

    if (rowMags[maxRow] > MinMagnitude && columnMags[maxColumn] > MinMagnitude) {
        char detectedTone;
        if (maxRow == 0) {
            if (maxColumn == 0) detectedTone = '1';
            else if (maxColumn == 1) detectedTone = '2';
            else if (maxColumn == 2) detectedTone = '3';
            else detectedTone = 'A';
        }
        else if (maxRow == 1) {
            if (maxColumn == 0) detectedTone = '4';
            else if (maxColumn == 1) detectedTone = '5';
            else if (maxColumn == 2) detectedTone = '6';
            else detectedTone = 'B';
        }
        else if (maxRow == 2) {
            if (maxColumn == 0) detectedTone = '7';
            else if (maxColumn == 1) detectedTone = '8';
            else if (maxColumn == 2) detectedTone = '9';
            else detectedTone = 'C';
        }
        else {
            if (maxColumn == 0) detectedTone = 'E';
            else if (maxColumn == 1) detectedTone = '0';
            else if (maxColumn == 2) detectedTone = 'F';
            else detectedTone = 'D';
        }

        std::cout << "Detected DTMF Tone: " << detectedTone << std::endl;
        return true;
    }
    return false;
}

// Function to process the file and detect DTMF tones in chunks
void Audio::processFile(const std::string& filename, int sampleRate, int bufferSize) {
    std::ifstream inFile(filename);
    if (!inFile) {
        std::cerr << "Unable to open file " << filename << std::endl;
        return;
    }

    while (true) {
        auto data = readDTMFDataChunk(inFile, bufferSize);
        if (data.empty()) break;
        analyzeDataWithGoertzel(data, sampleRate);
    }

    inFile.close();
}
