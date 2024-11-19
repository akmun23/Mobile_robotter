#ifndef ALGORITHMCOMPARETEST_H
#define ALGORITHMCOMPARETEST_H
#include "fft.h"
#include "dft.h"
#include "goertzel.h"
#include <iostream>
#include <chrono>

std::chrono::duration<double>  Maxtime{0};
std::chrono::duration<double> average{0};

int RunCompareTest() {
    int sampleRate = 44100;
    std::string filename = "/home/jonathan/Documents/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/Recording.txt";
    int bufferSize = 1500;


    // Read DTMF data from file
    auto data = readDTMFData(filename, sampleRate);
    if (data.empty()) {
        std::cerr << "Error: No data read from file!" << std::endl;
        return 3;
    }

    // Process the data in chunks of size 3000
    int numChunks = (data.size() + bufferSize - 1) / bufferSize; // Round up to cover remaining samples if any
    for (int chunk = 0; chunk < numChunks; ++chunk) {
        int startIndex = chunk * bufferSize;
        int endIndex = std::min(startIndex + bufferSize, static_cast<int>(data.size()));

        // Create a chunk of data to process
        std::vector<std::complex<double>> chunkData(data.begin() + startIndex, data.begin() + endIndex);

        // Zero-padding to make the chunk size equal to the next power of two
        int n = chunkData.size();
        int log2n = std::ceil(std::log2(n));
        int paddedSize = 1 << log2n;
        chunkData.resize(paddedSize, { 0.0, 0.0 }); // Zero pad if necessary

        std::vector<std::complex<double>> fftResult(paddedSize);

        // Start timer for FFT computation
       /* auto startFFT = std::chrono::high_resolution_clock::now();

        // Perform FFT on the chunk
        fft(chunkData.begin(), fftResult.begin(), log2n);

        // End timer for FFT computation
        auto endFFT = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> durationFFT = endFFT - startFFT;
        std::cout << "Time taken for FFT computation (chunk " << chunk + 1 << "): " << durationFFT.count() << " seconds." << std::endl;

        // Start timer for frequency analysis
        auto startFreqAnalysis = std::chrono::high_resolution_clock::now();

        // Find dominant frequencies in the chunk
        auto dominantFrequencies = findDominantFrequencies(fftResult, sampleRate);

        // End timer for frequency analysis
        auto endFreqAnalysis = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> durationFreqAnalysis = endFreqAnalysis - startFreqAnalysis;
        std::cout << "Time taken for frequency analysis (chunk " << chunk + 1 << "): " << durationFreqAnalysis.count() << " seconds." << std::endl;

        // Display the most dominant frequencies for the chunk
        std::cout << "Dominant Frequencies (Hz)\tMagnitude (Chunk " << chunk + 1 << ")\n";
        for (int i = 0; i < std::min(2, static_cast<int>(dominantFrequencies.size())); ++i) {
            std::cout << dominantFrequencies[i].first << "\t" << dominantFrequencies[i].second << std::endl;
        }*/

        // Now process the DFT of the chunk
        std::vector<std::complex<double>> dftResult(paddedSize);

        // Start timer for DFT computation
        auto startDFT = std::chrono::high_resolution_clock::now();

        // Assuming chunkData is of type std::vector<std::complex<double>>
        std::vector<double> realChunkData(chunkData.size());
        std::transform(chunkData.begin(), chunkData.end(), realChunkData.begin(), [](const std::complex<double>& c) {
            return c.real();
        });

        computeDFT(realChunkData, dftResult);



        // End timer for DFT computation
        auto endDFT = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> durationDFT = endDFT - startDFT;
        Maxtime += durationDFT;
        std::cout << "Time taken for DFT computation (chunk " << chunk + 1 << "): " << durationDFT.count() << " seconds." << std::endl;

        // Start timer for frequency analysis of DFT
        auto startDFTFreqAnalysis = std::chrono::high_resolution_clock::now();

        // Find dominant frequencies from DFT result
        auto DFTdominantFrequencies = findDominantFrequency(dftResult, sampleRate);

        //Process DTMF Tones #NEW
        auto detectedTonesSets = trueDTMF(DFTdominantFrequencies);


        // End timer for frequency analysis of DFT
        auto endDFTFreqAnalysis = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> DFTdurationFreqAnalysis = endDFTFreqAnalysis - startDFTFreqAnalysis;
        std::cout << "Time taken for DFT frequency analysis (chunk " << chunk + 1 << "): " << DFTdurationFreqAnalysis.count() << " seconds." << std::endl;


        for (size_t i = 0; i < detectedTonesSets.size(); ++i) {
            std::cout << "Detected DTMF tone set " << i + 1 << ": ";
            std::string toneSet;
            // Ensure detectedTonesSets[i] is a string and directly print each character
            toneSet = detectedTonesSets[i]; // Ensure we are working with a string
            for (size_t j = 0; j < toneSet.length(); ++j) {  // Iterate over each character in the string
                std::cout << toneSet[j] << " ";
            }
            std::cout << std::endl;
        }


       /* // Display the most dominant frequencies from DFT
        std::cout << "Dominant Frequencies (Hz)\tMagnitude (DFT chunk " << chunk + 1 << ")\n";
        for (int i = 0; i < std::min(2, static_cast<int>(DFTdominantFrequencies.size())); ++i) {
            int frequency = DFTdominantFrequencies[i];
            auto  detectedTones = trueDTMF(DFTdominantFrequencies);

            // Find the corresponding index in the DFT result
            int index = frequency * paddedSize / sampleRate;

            // Ensure the index is within bounds before accessing the result
            if (index >= 0 && index < dftResult.size()) {
                std::cout << frequency << "\t" << std::abs(dftResult[index]) << std::endl;
                std::cout << "Detected DTMF tones: ";
                   for (char tone : detectedTones) {
                       std::cout << tone << " ";
                   }
                   std::cout << std::endl;
            }
        }*/
    }
    average = Maxtime/numChunks;
    std::cout << "The average time for Chunk processing is " << average.count() << std::endl;



    // Goertzel algorithm

    GoertzelTesting GoertzelProcessor;
    GoertzelProcessor.processFileTest(filename, sampleRate, bufferSize);

    return 0;
}
#endif // ALGORITHMCOMPARETEST_H
