#include "FFT.h"
#include <fstream>
#include <iostream>
#include <cmath>



unsigned int bitReverse(unsigned int x, int log2n) {
    int n = 0;
    for (int i = 0; i < log2n; i++) {
        n <<= 1;
        n |= (x & 1);
        x >>= 1;
    }
    return n;
}

std::vector<std::complex<double>> readDTMFData(const std::string& filename, int& sampleRate) {
    std::ifstream inputFile(filename, std::ios::binary | std::ios::ate);
    std::vector<std::complex<double>> data;
    if (!inputFile) {
        std::cerr << "Error opening file!" << std::endl;
        return data;
    }

    double realPart;
    inputFile.seekg(0, std::ios::beg);
    while (inputFile >> realPart) {
        data.emplace_back(realPart, 0.0);
    }

    return data;
}

std::vector<std::pair<int, double>> findDominantFrequencies(const std::vector<std::complex<double>>& fftResult, int sampleRate) {
    std::vector<int> dtmfFrequencies = { 697, 770, 852, 941, 1209, 1336, 1477, 1633 };
    std::unordered_map<int, double> freqMagMap;

    int n = fftResult.size();
    for (int i = 0; i < n / 2; ++i) {
        int frequency = i * sampleRate / n;
        double magnitude = std::abs(fftResult[i]);

        if (std::find(dtmfFrequencies.begin(), dtmfFrequencies.end(), frequency) != dtmfFrequencies.end()) {
            if (freqMagMap.find(frequency) == freqMagMap.end() || magnitude > freqMagMap[frequency]) {
                freqMagMap[frequency] = magnitude;
            }
        }
    }

    std::vector<std::pair<int, double>> sortedFreqMags(freqMagMap.begin(), freqMagMap.end());
    std::sort(sortedFreqMags.begin(), sortedFreqMags.end(), [](const auto& a, const auto& b) {
        return a.second > b.second;
        });

    return sortedFreqMags;
}
