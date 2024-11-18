#include "fft.h"
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <iterator>
#include <algorithm>

// Constructor for FFTProcessing class
// Initializes member variables
FFTProcessing::FFTProcessing(int minMagnitude, const std::vector<int>& dtmfRowFrequencies, const std::vector<int>& dtmfColumnFrequencies, int frequencyTolerance)
    : minMagnitude(minMagnitude), dtmfRowFrequencies(dtmfRowFrequencies), dtmfColumnFrequencies(dtmfColumnFrequencies), frequencyTolerance(frequencyTolerance),
    timeToReadTone(0.000234 * 4), timeToSendMessage(0.000234 * 4 * 7), letterReceived(false), startOfMessageReceived(false) {}

// Function to reverse bits for FFT
unsigned int FFTProcessing::bitReverse(unsigned int x, int log2n) {
    int n = 0;
    for (int i = 0; i < log2n; i++) {
        n <<= 1;
        n |= (x & 1);
        x >>= 1;
    }
    return n;
}

// Function to perform FFT
std::vector<std::complex<double>> FFTProcessing::fft(const std::vector<std::complex<double>>& input, int log2n) {
    int n = 1 << log2n;
    std::vector<std::complex<double>> a(n);
    // Rearrange input array elements in bit-reversed order
    for (int i = 0; i < n; ++i) {
        a[bitReverse(i, log2n)] = input[i];
    }
    // FFT computation using the Cooley-Tukey algorithm
    for (int s = 1; s <= log2n; ++s) {
        int m = 1 << s;
        std::complex<double> wm = std::exp(std::complex<double>(0, -2 * PI / m));
        for (int k = 0; k < n; k += m) {
            std::complex<double> w = 1;
            for (int j = 0; j < m / 2; ++j) {
                std::complex<double> t = w * a[k + j + m / 2];
                std::complex<double> u = a[k + j];
                a[k + j] = u + t;
                a[k + j + m / 2] = u - t;
                w *= wm;
            }
        }
    }
    return a;
}

// Function to read DTMF data from file in chunks
std::vector<std::complex<double>> FFTProcessing::readDTMFDataChunk(std::ifstream& inFile, int bufferSize) {
    std::vector<std::complex<double>> data;
    double realPart;

    // Read data from file until buffer is full
    for (int i = 0; i < bufferSize && inFile >> realPart; ++i) {
        data.emplace_back(realPart, 0.0);
    }

    return data;
}

// Function to find dominant frequencies in FFT result
std::vector<std::pair<int, double>> FFTProcessing::findDominantFrequencies(const std::vector<std::complex<double>>& fftResult, int sampleRate) {
    std::unordered_map<int, double> freqMagMap;

    int n = fftResult.size();
    // Calculate frequency and magnitude for each FFT result
    for (int i = 0; i < n / 2; ++i) {
        int frequency = i * sampleRate / n;
        double magnitude = std::abs(fftResult[i]);

        // Check if the frequency is a DTMF frequency and update magnitude if it's higher
        if (magnitude > minMagnitude &&
            (std::find(dtmfRowFrequencies.begin(), dtmfRowFrequencies.end(), frequency) != dtmfRowFrequencies.end() ||
                std::find(dtmfColumnFrequencies.begin(), dtmfColumnFrequencies.end(), frequency) != dtmfColumnFrequencies.end())) {
            freqMagMap[frequency] = magnitude;
        }
    }

    // Sort frequencies by magnitude in descending order
    std::vector<std::pair<int, double>> sortedFreqMags(freqMagMap.begin(), freqMagMap.end());
    std::sort(sortedFreqMags.begin(), sortedFreqMags.end(), [](const auto& a, const auto& b) {
        return a.second > b.second;
        });

    return sortedFreqMags;
}

// Function to check if a frequency is approximately a DTMF row or column frequency
bool FFTProcessing::isApproximateDTMFRowOrColumnFrequency(int freq) {
    for (int dtmfFreq : dtmfRowFrequencies) {
        if (std::abs(freq - dtmfFreq) <= frequencyTolerance) {
            return true;
        }
    }
    for (int dtmfFreq : dtmfColumnFrequencies) {
        if (std::abs(freq - dtmfFreq) <= frequencyTolerance) {
            return true;
        }
    }
    return false;
}

// Function to calculate elapsed time
double FFTProcessing::timePassed(std::chrono::high_resolution_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
}

// Function to save detected DTMF signals and manage message state
bool FFTProcessing::saveSignal(std::vector<double> rowMags, std::vector<double> columnMags, int maxRow, int maxColumn) {
    if (rowMags[maxRow] > minMagnitude && columnMags[maxColumn] > minMagnitude && !letterReceived && ((maxRow == 3 && maxColumn == 0) || startOfMessageReceived)) {
        letterReceived = true;
        // Save detected tone based on DTMF frequency mapping
        if (maxRow == 0) {
            if (maxColumn == 0) receivedSignal.push_back('1');
            else if (maxColumn == 1) receivedSignal.push_back('2');
            else if (maxColumn == 2) receivedSignal.push_back('3');
            else if (maxColumn == 3) receivedSignal.push_back('A');
        }
        else if (maxRow == 1) {
            if (maxColumn == 0) receivedSignal.push_back('4');
            else if (maxColumn == 1) receivedSignal.push_back('5');
            else if (maxColumn == 2) receivedSignal.push_back('6');
            else if (maxColumn == 3) receivedSignal.push_back('B');
        }
        else if (maxRow == 2) {
            if (maxColumn == 0) receivedSignal.push_back('7');
            else if (maxColumn == 1) receivedSignal.push_back('8');
            else if (maxColumn == 2) receivedSignal.push_back('9');
            else if (maxColumn == 3) receivedSignal.push_back('C');
        }
        else if (maxRow == 3) {
            if (maxColumn == 0) {
                if (!startOfMessageReceived) {
                    clockStartMessage = std::chrono::high_resolution_clock::now();
                    clockStartTone = std::chrono::high_resolution_clock::now();
                }
                startOfMessageReceived = true;
                receivedSignal.push_back('*');
            }
            else if (maxColumn == 1) receivedSignal.push_back('0');
            else if (maxColumn == 2) receivedSignal.push_back('#');
            else if (maxColumn == 3) receivedSignal.push_back('D');
        }
        return true;
    }
    else if (letterReceived && ((timePassed(clockStartTone) + timeToReadTone / 4) > timeToReadTone)) {
        letterReceived = false;
        clockStartTone = std::chrono::high_resolution_clock::now();
    }
    return false;
}

void FFTProcessing::displayReceivedSignal() {
    if (!receivedSignal.empty()) {
        std::cout << "_____________\n";
        for (char ch : receivedSignal) {
            std::cout << ch;
        }
        std::cout << "\n_____________\n";
    }
    receivedSignal.clear();
    startOfMessageReceived = false;
    clockStartMessage = std::chrono::high_resolution_clock::now();
}

void FFTProcessing::processFile(const std::string& filename, int sampleRate, int bufferSize) {
    // Open the input file
    std::ifstream inFile(filename);
    if (!inFile) {
        std::cerr << "Unable to open file " << filename << std::endl;
        return;
    }

    std::vector<std::complex<double>> data;
    int numChunks = 0;
    double totalDuration = 0;
    double maxDuration = 0;

    // Process the file in chunks
    while (true) {
        // Start timer for chunk processing
        auto start = std::chrono::high_resolution_clock::now();

        // Read a chunk of data from the file
        data = readDTMFDataChunk(inFile, bufferSize);
        if (data.empty()) break;

        // Determine the size and the nearest power of 2 for FFT
        int n = data.size();
        int log2n = std::ceil(std::log2(n));
        int paddedSize = 1 << log2n;
        data.resize(paddedSize, { 0.0, 0.0 });

        // Perform FFT on the chunk of data
        std::vector<std::complex<double>> fftResult = fft(data, log2n);

        // Find dominant frequencies in the FFT result
        auto dominantFrequencies = findDominantFrequencies(fftResult, sampleRate);

        // End timer for chunk processing and calculate duration
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        totalDuration += duration.count();
        if (duration.count() > maxDuration) {
            maxDuration = duration.count();
        }
        numChunks++;

        // Initialize vectors to store row and column magnitudes
        std::vector<double> rowMags(dtmfRowFrequencies.size(), 0.0);
        std::vector<double> columnMags(dtmfColumnFrequencies.size(), 0.0);

        // Populate row and column magnitudes based on detected frequencies
        for (const auto& freqPair : dominantFrequencies) {
            int freq = freqPair.first;
            double magnitude = freqPair.second;

            auto rowIt = std::find(dtmfRowFrequencies.begin(), dtmfRowFrequencies.end(), freq);
            if (rowIt != dtmfRowFrequencies.end()) {
                int index = std::distance(dtmfRowFrequencies.begin(), rowIt);
                rowMags[index] = magnitude;
            }

            auto colIt = std::find(dtmfColumnFrequencies.begin(), dtmfColumnFrequencies.end(), freq);
            if (colIt != dtmfColumnFrequencies.end()) {
                int index = std::distance(dtmfColumnFrequencies.begin(), colIt);
                columnMags[index] = magnitude;
            }
        }

        // Identify the maximum row and column magnitudes
        int maxRow = std::distance(rowMags.begin(), std::max_element(rowMags.begin(), rowMags.end()));
        int maxColumn = std::distance(columnMags.begin(), std::max_element(columnMags.begin(), columnMags.end()));

        // Save the detected signal
        if (saveSignal(rowMags, columnMags, maxRow, maxColumn)) {
            std::cout << "DTMF tone detected in chunk " << numChunks << "!\n";
        }
        else {
            std::cout << "No DTMF tone detected in chunk " << numChunks << ".\n";
        }

        // Display the received signal if a complete DTMF message is detected
        if (receivedSignal.size() == 6 && receivedSignal.front() == '*' && receivedSignal.back() == '#') {
            displayReceivedSignal();
        }
    }

    // Calculate and display the average and maximum duration for processing chunks
    double averageDuration = totalDuration / numChunks;
    std::cout << "Average time taken for processing chunks: " << averageDuration << " seconds." << std::endl;
    std::cout << "Maximum time taken for processing a chunk: " << maxDuration << " seconds." << std::endl;

    // Close the input file
    inFile.close();
}
