#include "fft.h"
#include <fstream>
#include <iostream>
#include <unistd.h>


// Constructor for FFTProcessing class
// Initializes member variables
FFTProcessing::FFTProcessing(double minMagnitude, double timeToReadTone, int frequencyTolerance)
: MagnitudeAnalysis(minMagnitude, timeToReadTone), _minMagnitude(minMagnitude), _frequencyTolerance(frequencyTolerance) {}

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
    _TimeForCalculationStartFFT = std::chrono::high_resolution_clock::now();
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


    double calcTime = timePassed(_TimeForCalculationStartFFT);
    _TimeSumCalculationFFT += calcTime;
    if (calcTime > _calcTimeMaxFFT) {
        _calcTimeMaxFFT = calcTime;
    }
    if (_calcTimeMinFFT == 0 || calcTime < _calcTimeMinFFT) {
        _calcTimeMinFFT = calcTime;
    }
    usleep(200000);
    return a;
}

// Function to read DTMF data from file in chunks
std::vector<double> FFTProcessing::readDTMFData(std::ifstream& inFile, int bufferSize) {
    std::vector<double> signal;
    signal.resize(bufferSize);
    double value;
    for (int i = 0; i < bufferSize; ++i) {

        if(inFile >> value){
            signal[i] = value;
        }else {
            signal.resize(i);
            break;
        }
    }
    return signal;
}


// Function to find dominant frequencies in FFT result
std::vector<double> FFTProcessing::findDominantFrequencies(const std::vector<std::complex<double>>& fftResult, int sampleRate) {

    std::vector<int> Tones = {697, 770, 852, 941, 1209, 1336, 1477, 1633};
    std::vector<double> TonesMaxMag = {0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<double> magnitudes;

    int N = fftResult.size();
    for (int i = 0; i < N / 2; ++i) {
        double frequency = i * sampleRate / N;
        bool withinBounds = false;

        for (int ii = 0; ii < Tones.size(); ++ii) {
            if((frequency >= (Tones[ii]-_frequencyTolerance)) && (frequency <= (Tones[ii]+_frequencyTolerance))){
                withinBounds = true;
                break;
            }
        }

        if (withinBounds) {
            magnitudes.push_back(std::abs(fftResult[i]));
        } else {
            magnitudes.push_back(0); // Set magnitude to zero if frequency is out of range
        }
    }

    for (int i = 0; i < magnitudes.size(); ++i) {

        double currentFreg = i * sampleRate / N;

        for (int ii = 0; ii < Tones.size(); ++ii) {
            if((Tones[ii]-_frequencyTolerance) < currentFreg && currentFreg < (Tones[ii]+_frequencyTolerance)){
                if(magnitudes[i] > TonesMaxMag[ii] && magnitudes[i] > _minMagnitude){
                    TonesMaxMag[ii] = magnitudes[i];
                }
            }
        }
    }
    return TonesMaxMag;

}

std::vector<double> FFTProcessing::processFile(std::ifstream& file, int sampleRate, int bufferSize) {
    _TimeForEntireSequenceStartFFT = std::chrono::high_resolution_clock::now();
    std::ofstream outputFileFFT;
    outputFileFFT.open("FFT_Test_Output.txt", std::ios_base::trunc); // The file is opened in append mode meaning that the data will be added to the end of the file
    outputFileFFT.close();
    outputFileFFT.open("FFT_Test_Output.txt", std::ios_base::app); // The file is opened in append mode meaning that the data will be added to the end of the file
    outputFileFFT << "New sequence of messages" << std::endl;

    while(true){
        std::chrono::high_resolution_clock::time_point TimeForChunkStartFFT = std::chrono::high_resolution_clock::now();
        _data = readDTMFData(file, bufferSize);
        if(_data.size() < bufferSize){
            break;
        }
        std::vector<std::complex<double>> complexData;
        for (int i = 0; i < _data.size(); ++i) {
            complexData.push_back({ _data[i], 0.0 });
        }


        // Zero-padding to make the chunk size equal to the next power of two
        int n = complexData.size();
        int log2n = std::ceil(std::log2(n));    // Finds the lowest integer value to put to the power of 2 to get n or higher
        int paddedSize = 1 << log2n;            // Finds the 2^log2n value by bit shifting 1 by log2n
        complexData.resize(paddedSize, { 0.0, 0.0 }); // Zero pad if necessary


        std::vector<std::complex<double>> fftResult(paddedSize);

        // Perform FFT on the chunk
        fftResult = fft(complexData, log2n);


        std::vector<double> dominantFrequencies = findDominantFrequencies(fftResult, sampleRate); // START HER DEN KAN IKKE FINDE HASHTAGS

        analyseMagnitudes(dominantFrequencies);

        checkMessageState(outputFileFFT, _correctMessages, _incorrectMessages, _messageCounter);

        _countFFT++;
    }
    outputFileFFT.close();
    _calculationTime = timePassed(_TimeForEntireSequenceStartFFT);
    double avgCalcTime = _TimeSumCalculationFFT/_countFFT;

    return checkOutputFile("FFT_Test_Output.txt", _calculationTime, "Checked_Output_FFT.txt", avgCalcTime, _calcTimeMaxFFT, _calcTimeMinFFT);
}



std::vector<double> FFTProcessing::processSound(float* input, int sampleRate, int bufferSize){



       std::vector<std::complex<double>> complexData;
       for (int i = 0; i < bufferSize; ++i) {
           complexData.push_back({ input[i], 0.0 });
       }
       // Zero-padding to make the chunk size equal to the next power of two
       int n = complexData.size();
       int log2n = std::ceil(std::log2(n));    // Finds the lowest integer value to put to the power of 2 to get n or higher
       int paddedSize = 1 << log2n;            // Finds the 2^log2n value by bit shifting 1 by log2n
       complexData.resize(paddedSize, { 0.0, 0.0 }); // Zero pad if necessary
       std::vector<std::complex<double>> fftResult(paddedSize);
       // Perform FFT on the chunk
       fftResult = fft(complexData, log2n);
       std::vector<double> dominantFrequencies = findDominantFrequencies(fftResult, sampleRate); // START HER DEN KAN IKKE FINDE HASHTAGS


       return dominantFrequencies;
}
