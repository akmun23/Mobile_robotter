#ifndef ALGORITHMCOMPARETEST_H
#define ALGORITHMCOMPARETEST_H
#include "fft.h"
#include "dft.h"
#include "goertzel.h"
#include <iostream>
#include <chrono>

std::chrono::duration<double> Maxtime{0};
std::chrono::duration<double> average{0};



int RunCompareTest() {
    int sampleRate = 44100;
    std::string filename = "/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/RecordingEchoShortDistance.txt";
    int bufferSize = 1500;

    std::vector<double> DFTResult;
    std::vector<double> FFTResult;
    std::vector<double> GoertzelResult;

    // Output Files
    std::ofstream bufferCalculationSpeed;
    bufferCalculationSpeed.open("bufferCalculationSpeed.txt");
    std::ofstream correctAndFailFile;
    correctAndFailFile.open("correctAndFailFile.txt");


    DFTResult = runDFT(filename, sampleRate, bufferSize);

    //FFT algorithm;
    FFTProcessing FFTProcessor(100, {697, 770, 852, 941}, {1209, 1336, 1477, 1633}, 20);
    FFTResult = FFTProcessor.processFile(filename, sampleRate, bufferSize);


    // Goertzel algorithm
    GoertzelTesting GoertzelProcessor;
    GoertzelResult = GoertzelProcessor.processFileTest(filename, sampleRate, bufferSize);

    // Compare the results
    std::cout << "Correct messages -  DFT: " << DFTResult[0] << " FFT: " << FFTResult[0] << " Goertzel: " << GoertzelResult[0] << std::endl;
    std::cout << "Incorrect messages -  DFT: " << DFTResult[1] << " FFT: " << FFTResult[1] << " Goertzel: " << GoertzelResult[1] << std::endl;
    std::cout << "Incorrect format -  DFT: " << DFTResult[2] << " FFT: " << FFTResult[2] << " Goertzel: " << GoertzelResult[2] << std::endl;
    std::cout << "Total messages -  DFT: " << DFTResult[3] << " FFT: " << FFTResult[3] << " Goertzel: " << GoertzelResult[3] << std::endl;
    std::cout << "Correct format percentage -  DFT: " << DFTResult[4] << " FFT: " << FFTResult[4] << " Goertzel: " << GoertzelResult[4] << std::endl;
    std::cout << "Correct total messages -  DFT: " << DFTResult[5] << " FFT: " << FFTResult[5] << " Goertzel: " << GoertzelResult[5] << std::endl;
    std::cout << "Average time for tone calculation -  DFT: " << DFTResult[6] << " FFT: " << FFTResult[6] << " Goertzel: " << GoertzelResult[6] << std::endl;

    // Write to file
    bufferCalculationSpeed << DFTResult[6] << std::endl;
    bufferCalculationSpeed << FFTResult[6] << std::endl;
    bufferCalculationSpeed << GoertzelResult[6] << std::endl;

    correctAndFailFile << DFTResult[0] << std::endl << DFTResult[1] << std::endl << DFTResult[2] << std::endl << DFTResult[3] << std::endl << DFTResult[4] << std::endl << DFTResult[5] << std::endl << std::endl;
    correctAndFailFile << FFTResult[0] << std::endl << FFTResult[1] << std::endl << FFTResult[2] << std::endl << FFTResult[3] << std::endl << FFTResult[4] << std::endl << FFTResult[5] << std::endl << std::endl;
    correctAndFailFile << GoertzelResult[0] << std::endl << GoertzelResult[1] << std::endl << GoertzelResult[2] << std::endl << GoertzelResult[3] << std::endl << GoertzelResult[4] << std::endl << GoertzelResult[5] << std::endl;


    return 0;

}
#endif // ALGORITHMCOMPARETEST_H
