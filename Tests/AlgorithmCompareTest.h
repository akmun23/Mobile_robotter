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
    std::string filename = "/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/Recording.txt";
    int bufferSize = 1500;

    runDFT(filename, sampleRate, bufferSize);

    //FFT algorithm;
    FFTProcessing FFTProcessor(100, {697, 770, 852, 941}, {1209, 1336, 1477, 1633}, 20);
    FFTProcessor.processFile(filename, sampleRate, bufferSize);


    // Goertzel algorithm
    GoertzelTesting GoertzelProcessor;
    GoertzelProcessor.processFileTest(filename, sampleRate, bufferSize);
    return 0;
}
#endif // ALGORITHMCOMPARETEST_H
