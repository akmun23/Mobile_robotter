#ifndef ALGORITHMCOMPARETEST_H
#define ALGORITHMCOMPARETEST_H
#include "fft.h"
#include "dft.h"
#include "goertzel.h"
#include <iostream>
#include <chrono>
#include "magnitudeanalysis.h"

std::chrono::duration<double> Maxtime{0};
std::chrono::duration<double> average{0};

void TimePrinter(std::chrono::high_resolution_clock::time_point start){
    std::chrono::duration<double> TimeElapsed = std::chrono::high_resolution_clock::now() - start;
    std::cout << "Time passed: " << TimeElapsed.count() << "s."<< std::endl;

}

int RunCompareTest() {
    int sampleRate = 44100;
    int bufferSize = 750;       // 750 works good
    double AudieBufferSize = 6350.4;

    int DFTMagnitude = 3;       // 5 works good
    int FFTMagnitude = 3;       // 5 works good
    int GoertzelMagnitude = 60; // 60 works good

    double TimeToCalcDFT = 0.205;
    double TimeToCalcFFT = 0.205;
    double TimeToCalcGoertzelWithDelay = 0.21;

    double scalingFactor = 0.9;
    double TimeToReadToneDFT = TimeToCalcDFT*(AudieBufferSize/bufferSize)*scalingFactor;
    double timeToReadToneFFT = TimeToCalcFFT*(AudieBufferSize/bufferSize)*scalingFactor;
    double timeToReadToneGoertzel = TimeToCalcGoertzelWithDelay*(AudieBufferSize/bufferSize)*scalingFactor;

    int FFTTolerance = 20;      // 20 works good

    // Input files
    std::vector<std::pair<int,std::string>> TestfileNames;
    TestfileNames.push_back({1,"/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/RecordingShortVersion.txt"});
    //TestfileNames.push_back({1,"/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/Recording.txt"});
    //TestfileNames.push_back({1,"/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/RecordingEchoShortDistance.txt"});
    //TestfileNames.push_back({1,"/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/RecordingNoEchoShortDistance.txt"});
    //TestfileNames.push_back({1,"/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/RecordingEchoLongDistance.txt"});
    //TestfileNames.push_back({1,"/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/RecordingNoEchoLongDistance.txt"});
    std::vector<std::pair<int,std::string>> DFTTestfileNames = TestfileNames;
    std::vector<std::pair<int,std::string>> FFTTestfileNames = TestfileNames;
    std::vector<std::pair<int,std::string>> GoertzelTestfileNames = TestfileNames;
    //GoertzelTestfileNames.push_back({1,"/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/Recording1ChannelsShort.txt"});
    //GoertzelTestfileNames.push_back({2,"/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/Recording2ChannelsShort.txt"});



    // Output Files
    std::ofstream DFT_buffer_CalculationSpeed;
    std::ofstream FFT_buffer_CalculationSpeed;
    std::ofstream Goertzel_buffer_CalculationSpeed;

    std::ofstream DFT_correctAndFailFile;
    std::ofstream FFT_correctAndFailFile;
    std::ofstream Goertzel_correctAndFailFile;

    DFT_buffer_CalculationSpeed.open("DFT_buffer_CalculationSpeed.txt");
    FFT_buffer_CalculationSpeed.open("FFT_buffer_CalculationSpeed.txt");
    Goertzel_buffer_CalculationSpeed.open("Goertzel_buffer_CalculationSpeed.txt");

    DFT_correctAndFailFile.open("DFT_correctAndFailFile.txt");
    FFT_correctAndFailFile.open("FFT_correctAndFailFile.txt");
    Goertzel_correctAndFailFile.open("Goertzel_correctAndFailFile.txt");

    // Data for the different algorithms
    std::vector<double> DFTResult;
    std::vector<double> FFTResult;
    std::vector<double> GoertzelResult;

    std::vector<double> DFTTime;
    std::vector<double> FFTTime;
    std::vector<double> GoertzelTime;

    std::vector<double> DFTCorrect;
    std::vector<double> FFTCorrect;
    std::vector<double> GoertzelCorrect;

    std::vector<double> DFTIncorrect;
    std::vector<double> FFTIncorrect;
    std::vector<double> GoertzelIncorrect;

    std::vector<double> DFTIncorrectFormat;
    std::vector<double> FFTIncorrectFormat;
    std::vector<double> GoertzelIncorrectFormat;

    std::vector<double> DFTTotal;
    std::vector<double> FFTTotal;
    std::vector<double> GoertzelTotal;

    std::vector<double> DFTCorrectFormat;
    std::vector<double> FFTCorrectFormat;
    std::vector<double> GoertzelCorrectFormat;

    std::vector<double> DFTCorrectTotal;
    std::vector<double> FFTCorrectTotal;
    std::vector<double> GoertzelCorrectTotal;
    double DFTMaxTime = 0;
    double DFTMinTime = 0;
    double FFTMaxTime = 0;
    double FFTMinTime = 0;
    double GoertzelMaxTime = 0;
    double GoertzelMinTime = 0;


    std::chrono::high_resolution_clock::time_point clockStart = std::chrono::high_resolution_clock::now();
    // Loop through the files

    std::cout << "Start of DFT" << std::endl;
    for (int i = 0; i < DFTTestfileNames.size(); ++i) {
        std::ifstream file;
        file.open(DFTTestfileNames[i].second);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << DFTTestfileNames[i].second << std::endl;
            return 1;
        }

        // DFT algorithm
        DFT DFTProcessor(DFTMagnitude, TimeToReadToneDFT);
        DFTResult = DFTProcessor.runDFT(file, sampleRate, bufferSize);
        std::cout << "DFT File: "<< i+1 << " Processed" << std::endl;
        TimePrinter(clockStart);
        // Time
        DFTTime.push_back(DFTResult[6]);
        if (DFTResult[7] > DFTMaxTime){
            DFTMaxTime = DFTResult[7];
        }
        if (DFTResult[8] < DFTMinTime || i == 0){
            DFTMinTime = DFTResult[8];
        }

        // Correct and incorrect
        DFTCorrect.push_back(DFTResult[0]);
        DFTIncorrect.push_back(DFTResult[1]);
        DFTIncorrectFormat.push_back(DFTResult[2]);
        DFTTotal.push_back(DFTResult[3]);
        DFTCorrectFormat.push_back(DFTResult[4]);
        DFTCorrectTotal.push_back(DFTResult[5]);

        DFT_buffer_CalculationSpeed << DFTTime[i] << std::endl;
        DFT_correctAndFailFile << DFTCorrect[i] << " " << DFTIncorrect[i] << " " << DFTIncorrectFormat[i] << " " << DFTTotal[i] << " " << DFTCorrectFormat[i] << " " << DFTCorrectTotal[i] << std::endl;

    }
    DFT_buffer_CalculationSpeed << DFTMaxTime << std::endl;
    DFT_buffer_CalculationSpeed << DFTMinTime << std::endl;

    std::cout << "End of DFT" << std::endl;

    std::cout << "Start of FFT" << std::endl;


    for (int i = 0; i < FFTTestfileNames.size(); ++i){
        std::ifstream file;
        file.open(FFTTestfileNames[i].second);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << FFTTestfileNames[i].second << std::endl;
            return 1;
        }
        //FFT algorithm;
        FFTProcessing FFTProcessor(FFTMagnitude,timeToReadToneFFT, FFTTolerance);
        FFTResult = FFTProcessor.processFile(file, sampleRate, bufferSize);
        std::cout << "FFT File: "<< i+1 << " Processed" << std::endl;
        TimePrinter(clockStart);

        // Time
        FFTTime.push_back(FFTResult[6]);
        if (FFTResult[7] > FFTMaxTime){
            FFTMaxTime = FFTResult[7];
        }
        if (FFTResult[8] < FFTMinTime || i == 0){
            FFTMinTime = FFTResult[8];
        }

        // Correct and incorrect
        FFTCorrect.push_back(FFTResult[0]);
        FFTIncorrect.push_back(FFTResult[1]);
        FFTIncorrectFormat.push_back(FFTResult[2]);
        FFTTotal.push_back(FFTResult[3]);
        FFTCorrectFormat.push_back(FFTResult[4]);
        FFTCorrectTotal.push_back(FFTResult[5]);

        FFT_buffer_CalculationSpeed << FFTTime[i] << std::endl;
        FFT_correctAndFailFile << FFTCorrect[i] << " " << FFTIncorrect[i] << " " << FFTIncorrectFormat[i] << " " << FFTTotal[i] << " " << FFTCorrectFormat[i] << " " << FFTCorrectTotal[i] << std::endl;

    }
    FFT_buffer_CalculationSpeed << FFTMaxTime << std::endl;
    FFT_buffer_CalculationSpeed << FFTMinTime << std::endl;

    std::cout << "End of FFT" << std::endl;

    std::cout << "Start of Goertzel" << std::endl;

    for (int i = 0; i < GoertzelTestfileNames.size(); ++i) {
        std::ifstream file;
        file.open(GoertzelTestfileNames[i].second);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << GoertzelTestfileNames[i].second << std::endl;
            return 1;
        }

        // Goertzel algorithm
        GoertzelTesting GoertzelProcessor(GoertzelMagnitude, timeToReadToneGoertzel);
        GoertzelResult = GoertzelProcessor.processFile(file, sampleRate, bufferSize, GoertzelTestfileNames[i].first);
        std::cout << "Goertzel File: "<< i+1 << " Processed" << std::endl;
        TimePrinter(clockStart);

        // Time
        GoertzelTime.push_back(GoertzelResult[6]);
        if (GoertzelResult[7] > GoertzelMaxTime){
            GoertzelMaxTime = GoertzelResult[7];
        }
        if (GoertzelResult[8] < GoertzelMinTime || i == 0){
            GoertzelMinTime = GoertzelResult[8];
        }

        // Correct and incorrect
        GoertzelCorrect.push_back(GoertzelResult[0]);
        GoertzelIncorrect.push_back(GoertzelResult[1]);
        GoertzelIncorrectFormat.push_back(GoertzelResult[2]);
        GoertzelTotal.push_back(GoertzelResult[3]);
        GoertzelCorrectFormat.push_back(GoertzelResult[4]);
        GoertzelCorrectTotal.push_back(GoertzelResult[5]);


        Goertzel_buffer_CalculationSpeed << GoertzelTime[i] << std::endl;
        Goertzel_correctAndFailFile << GoertzelCorrect[i] << " " << GoertzelIncorrect[i] << " " << GoertzelIncorrectFormat[i] << " " << GoertzelTotal[i] << " " << GoertzelCorrectFormat[i] << " " << GoertzelCorrectTotal[i] << std::endl;
    }

    Goertzel_buffer_CalculationSpeed << GoertzelMaxTime << std::endl;
    Goertzel_buffer_CalculationSpeed << GoertzelMinTime << std::endl;


    std::cout << "End of Goertzel" << std::endl;


    DFT_buffer_CalculationSpeed.close();
    FFT_buffer_CalculationSpeed.close();
    Goertzel_buffer_CalculationSpeed.close();

    DFT_correctAndFailFile.close();
    FFT_correctAndFailFile.close();
    Goertzel_correctAndFailFile.close();

    return 0;

}
#endif // ALGORITHMCOMPARETEST_H
