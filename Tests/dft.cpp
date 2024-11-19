#include "dft.h"
#include <chrono>
#include <map>
#include<unordered_map>


struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

std::vector<double> readDTMFData(const std::string& filename, int sampleRate) {
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

//Funtion to get true DTMF #NEW
char getDTMFcharacter(int rowFreq, int colFreq){

    static const std::unordered_map<std::pair<int, int>, char, pair_hash> dtmfMap = {
        {{697, 1209}, '1'}, {{697, 1336}, '2'}, {{697, 1477}, '3'}, {{697, 1633}, 'A'},
        {{770, 1209}, '4'}, {{770, 1336}, '5'}, {{770, 1477}, '6'}, {{770, 1633}, 'B'},
        {{852, 1209}, '7'}, {{852, 1336}, '8'}, {{852, 1477}, '9'}, {{852, 1633}, 'C'},
        {{941, 1209}, '*'}, {{941, 1336}, '0'}, {{941, 1477}, '#'}, {{941, 1633}, 'D'}

    };
    auto it = dtmfMap.find({rowFreq, colFreq});
    return (it != dtmfMap.end()) ? it ->second : '\0';
}

//Functionto find DTMF message #NEW
std::vector<std::string> trueDTMF(const std::vector<double> &dominantFrequencies) {
    // Mapping of DTMF frequency pairs to characters
    std::map<std::pair<int, int>, char> dtmfMap = {
        {{697, 1209}, '1'}, {{697, 1336}, '2'}, {{697, 1477}, '3'},
        {{770, 1209}, '4'}, {{770, 1336}, '5'}, {{770, 1477}, '6'},
        {{852, 1209}, '7'}, {{852, 1336}, '8'}, {{852, 1477}, '9'},
        {{941, 1209}, '*'}, {{941, 1336}, '0'}, {{941, 1477}, '#'}
    };

    std::vector<std::string> dtmfToneSets; // Store sets of 6 DTMF tones
    std::string currentSet; // Accumulate tones in the current set

    // Iterate through dominant frequencies in pairs
    for (size_t i = 0; i + 1 < dominantFrequencies.size(); i += 2) {
        int freq1 = dominantFrequencies[i];
        int freq2 = dominantFrequencies[i + 1];

        // Normalize the order of the frequency pair
        if (freq1 > freq2) std::swap(freq1, freq2);

        // Check if the pair exists in the DTMF map
        auto it = dtmfMap.find({freq1, freq2});
        if (it != dtmfMap.end()) {
            currentSet += it->second; // Add the DTMF tone to the current set

            // If the current set reaches 6 tones, save it and start a new set
            if (currentSet.size() == 6) {
                dtmfToneSets.push_back(currentSet);
                currentSet.clear();
            }
        }
    }

    // Add any remaining tones as a partial set
    if (!currentSet.empty()) {
        dtmfToneSets.push_back(currentSet);
    }

    return dtmfToneSets;
}

