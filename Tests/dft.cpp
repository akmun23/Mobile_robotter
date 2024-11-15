#include "dft.h"

// Function to compute the DFT
void computeDFT(const std::vector<double>& input, std::vector<std::complex<double>>& output) {
    int N = input.size();
    output.resize(N);
    for (int k = 0; k < N; ++k) {
        std::complex<double> sum = 0.0;
        for (int n = 0; n < N; ++n) {
            double angle = 2 * PI_M * k * n / N;
            sum += input[n] * std::exp(-std::complex<double>(0.0, angle));
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
    std::vector<double> dtmfBounds = {
        697, 770, 852, 941, 951, 1209, 1336, 1477, 1633
    };

    for (int i = 0; i < N / 2; ++i) {
        double frequency = i * samplingRate / N;
        bool withinBounds = false;

        // Iterate through dtmfBounds and check the frequency
        for (size_t j = 0; j < dtmfBounds.size(); ++j) {
            if (j < dtmfBounds.size() - 1 && frequency >= dtmfBounds[j] && frequency < dtmfBounds[j + 1]) {
                withinBounds = true;
                break;
            }
        }

        if (withinBounds) {
            magnitudes.push_back(std::abs(dft[i]));
        }
        else {
            magnitudes.push_back(0); // Set magnitude to zero if frequency is out of range
        }
    }


    // Sort magnitudes and get top two frequencies
    std::vector<std::pair<double, int>> magIndexPairs;
    for (int i = 0; i < magnitudes.size(); ++i) {
        magIndexPairs.push_back({ magnitudes[i], i });
    }
    std::sort(magIndexPairs.rbegin(), magIndexPairs.rend());

    dominantFreqs.push_back(magIndexPairs[0].second * samplingRate / N);
    dominantFreqs.push_back(magIndexPairs[1].second * samplingRate / N);

    return dominantFreqs;
}

// Function to read DTMF data from file
std::vector<double> readDTMFData(const std::string& filename) {
    std::vector<double> signal;
    std::ifstream inFile(filename);

    if (!inFile) {
        std::cerr << "Unable to open file " << filename << std::endl;
        exit(1);   // call system to stop
    }

    double x;
    while (inFile >> x) {
        signal.push_back(x);
    }

    return signal;
}
