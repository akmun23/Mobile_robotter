#include <algorithm>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <complex>
#include <string>

// Function to compute the DFT
void computeDFT(const std::vector<double>& input, std::vector<std::complex<double>>& output) {
    int N = input.size();
    output.resize(N);
    for (int k = 0; k < N; ++k) {
        std::complex<double> sum = 0.0;
        for (int n = 0; n < N; ++n) {
            double angle = 2 * M_PI * k * n / N;
            sum += input[n] * std::exp(-std::complex<double>(0, angle));
        }
        output[k] = sum;
    }
}

// Function to find the dominant frequencies
std::vector<double> findDominantFrequency(const std::vector<std::complex<double>>& dft, double samplingRate) {
    int N = dft.size();
    std::vector<double> dominantFreqs;
    std::vector<double> magnitudes;

    // Define the frequency bounds for DTMF tones
    std::vector<std::pair<double, double>> dtmfBounds = {
        {687, 707}, {760, 780}, {842, 862}, {931, 951},
        {1199, 1219}, {1326, 1346}, {1467, 1487}, {1623, 1643}
    };

    for (int i = 0; i < N / 2; ++i) {
        double frequency = i * samplingRate / N;
        bool withinBounds = false;

        for (auto& bounds : dtmfBounds) {
            if (frequency >= bounds.first && frequency <= bounds.second) {
                withinBounds = true;
                break;
            }
        }

        if (withinBounds) {
            magnitudes.push_back(std::abs(dft[i]));
        } else {
            magnitudes.push_back(0); // Set magnitude to zero if frequency is out of range
        }
    }

    // Sort magnitudes and get top two frequencies
    std::vector<std::pair<double, int>> magIndexPairs;
    for (int i = 0; i < magnitudes.size(); ++i) {
        magIndexPairs.push_back({magnitudes[i], i});
    }
    std::sort(magIndexPairs.rbegin(), magIndexPairs.rend());

    dominantFreqs.push_back(magIndexPairs[0].second * samplingRate / N);
    dominantFreqs.push_back(magIndexPairs[1].second * samplingRate / N);

    return dominantFreqs;
}
// Function to read DTMF data from file
std::vector<double> readDTMFData(const std::string& filename) {
    std::vector<double> signal;
    std::ifstream inFile;


    inFile.open(filename);

    if (!inFile) {
        std::cerr << "Unable to open file datafile.txt";
        exit(1);   // call system to stop
    }
    double x;

    while (inFile >> x) {
        signal.push_back(x);
    }

    return signal;
}

// Main function
int main() {



    std::string filename = "/home/emil/workspace/fourier/DTMF9R.txt"; // Replace with your actual file path
    double samplingRate = 48000; // Example sampling rate in Hz

    // Read DTMF data from file
    std::vector<double> signal = readDTMFData(filename);

    std::vector<std::complex<double>> dft;
    int bufferSize = 3000;

    for (int k = 0; k < (signal.size()) / bufferSize; k++) {
        std::vector<double> temp;
        for (int kk = 0; kk < bufferSize; kk++) {
            temp.push_back(signal[bufferSize * k + kk]);
        }
        computeDFT(temp, dft);
        std::vector<double> dominantFrequency = findDominantFrequency(dft, samplingRate);
        std::cout << "Sample number: " << k << std::endl;
        std::cout << "Dominant Frequency: " << dominantFrequency[0] << " Hz" << std::endl;
        std::cout << "Second Dominant Frequency: " << dominantFrequency[1] << " Hz" << std::endl;
        std::cout << std::endl;
    }

    return 0;
}
