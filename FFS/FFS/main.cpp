#include "FFT.h"
#include "DFT.h"
#include <iostream>
#include <chrono>
#include <vector>
#include <complex>
#include <algorithm>
#include <cmath>

// Define DTMF row and column frequencies
const std::vector<int> DTMFRowFrequencies = { 697, 770, 852, 941 };
const std::vector<int> DTMFColumnFrequencies = { 1209, 1336, 1477, 1633 };
const int frequencyTolerance = 10; // Allowable tolerance in Hz

// Function to check if a frequency is approximately a DTMF row or column frequency
bool isApproximateDTMFRowOrColumnFrequency(int freq) {
    for (int dtmfFreq : DTMFRowFrequencies) {
        if (std::abs(freq - dtmfFreq) <= frequencyTolerance) {
            std::cout << "Frequency " << freq << " Hz is within tolerance of DTMF row frequency " << dtmfFreq << " Hz\n";
            return true;
        }
    }
    for (int dtmfFreq : DTMFColumnFrequencies) {
        if (std::abs(freq - dtmfFreq) <= frequencyTolerance) {
            std::cout << "Frequency " << freq << " Hz is within tolerance of DTMF column frequency " << dtmfFreq << " Hz\n";
            return true;
        }
    }
    return false;
}

int main() {
    int sampleRate = 48000;
    std::string filename = "C:\\Users\\Jonathan\\Desktop\\C++ Projekter\\SoftwareExamen\\Mobile_robotter\\GetInfoFromMic\\DTMF9R.txt";
    int bufferSize = 3000;

    // Read DTMF data from file
    auto data = readDTMFData(filename, sampleRate);
    if (data.empty()) {
        std::cerr << "Error: No data read from file!" << std::endl;
        return 3;
    }

    // Process the data in chunks
    int numChunks = (data.size() + bufferSize - 1) / bufferSize;
    for (int chunk = 0; chunk < numChunks; ++chunk) {
        int startIndex = chunk * bufferSize;
        int endIndex = std::min(startIndex + bufferSize, static_cast<int>(data.size()));

        // Create a chunk of data to process
        std::vector<std::complex<double>> chunkData(data.begin() + startIndex, data.begin() + endIndex);

        // Zero-padding to make the chunk size equal to the next power of two
        int n = chunkData.size();
        int log2n = std::ceil(std::log2(n));
        int paddedSize = 1 << log2n;
        chunkData.resize(paddedSize, { 0.0, 0.0 });

        std::vector<std::complex<double>> fftResult(paddedSize);

        // Start timer for FFT computation
        auto startFFT = std::chrono::high_resolution_clock::now();

        // Perform FFT on the chunk
        fft(chunkData.begin(), fftResult.begin(), log2n);

        // End timer for FFT computation
        auto endFFT = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> durationFFT = endFFT - startFFT;
        std::cout << "Time taken for FFT computation (chunk " << chunk + 1 << "): " << durationFFT.count() << " seconds." << std::endl;

        // Find dominant frequencies in the chunk
        auto dominantFrequencies = findDominantFrequencies(fftResult, sampleRate);

        // Store dominant frequencies in a vector
        std::vector<int> detectedFrequencies;
        for (const auto& freqPair : dominantFrequencies) {
            detectedFrequencies.push_back(freqPair.first);
        }

        if (detectedFrequencies.size() >= 2) {
            // Sort frequencies to get the smallest and largest
            std::sort(detectedFrequencies.begin(), detectedFrequencies.end());
            int rowFrequency = detectedFrequencies.front();
            int columnFrequency = detectedFrequencies.back();

            // Check if the frequencies match DTMF tones
            if (isApproximateDTMFRowOrColumnFrequency(rowFrequency) && isApproximateDTMFRowOrColumnFrequency(columnFrequency)) {
                std::cout << "DTMF tone detected in chunk " << chunk + 1 << "!\n";
                std::cout << "Row Frequency: " << rowFrequency << " Hz\n";
                std::cout << "Column Frequency: " << columnFrequency << " Hz\n";
            }
            else {
                std::cout << "No DTMF tone detected in chunk " << chunk + 1 << ".\n";
            }
        }

        // Prepare for DFT computation
        std::vector<std::complex<double>> dftResult(paddedSize);

        // Start timer for DFT computation
        auto startDFT = std::chrono::high_resolution_clock::now();

        // Perform DFT on the real part of chunkData
        std::vector<double> realChunkData(chunkData.size());
        std::transform(chunkData.begin(), chunkData.end(), realChunkData.begin(), [](const std::complex<double>& c) {
            return c.real();
            });
        computeDFT(realChunkData, dftResult);

        // End timer for DFT computation
        auto endDFT = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> durationDFT = endDFT - startDFT;
        std::cout << "Time taken for DFT computation (chunk " << chunk + 1 << "): " << durationDFT.count() << " seconds." << std::endl;

        // Find dominant frequencies from DFT result
        auto DFTdominantFrequencies = findDominantFrequency(dftResult, sampleRate);
        std::cout << "Dominant Frequencies (Hz)\tMagnitude (DFT chunk " << chunk + 1 << ")\n";
        for (int i = 0; i < std::min(2, static_cast<int>(DFTdominantFrequencies.size())); ++i) {
            int frequency = DFTdominantFrequencies[i];
            int index = frequency * paddedSize / sampleRate;
            if (index >= 0 && index < dftResult.size()) {
                std::cout << frequency << "\t" << std::abs(dftResult[index]) << std::endl;
            }
        }
    }

    return 0;
}
