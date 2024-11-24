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
    int bufferSize = 1500;

    // Input files
    std::vector<std::string> TestfileNames;
    //TestfileNames.push_back("/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/RecordingShortVersion.txt");
    //TestfileNames.push_back("/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/Recording.txt");
    TestfileNames.push_back("/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/RecordingEchoShortDistance.txt");
    TestfileNames.push_back("/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/RecordingNoEchoShortDistance.txt");
    TestfileNames.push_back("/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/RecordingEchoLongDistance.txt");
    TestfileNames.push_back("/home/pascal/Dokumenter/GitHub/Mobile_robotter/Tests/build/Desktop-Debug/RecordingNoEchoLongDistance.txt");

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

    // Loop through the files
    for (int i = 0; i < TestfileNames.size(); ++i) {
        std::ifstream file;
        file.open(TestfileNames[i]);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file " << TestfileNames[i] << std::endl;
            return 1;
        }

        // DFT algorithm
        DFTResult = runDFT(file, sampleRate, bufferSize);
        file.clear();
        file.seekg(0, std::ios::beg);


        //FFT algorithm;
        FFTProcessing FFTProcessor(100, {697, 770, 852, 941}, {1209, 1336, 1477, 1633}, 20);
        FFTResult = FFTProcessor.processFile(file, sampleRate, bufferSize);
        file.clear();
        file.seekg(0, std::ios::beg);


        // Goertzel algorithm
        GoertzelTesting GoertzelProcessor;
        GoertzelResult = GoertzelProcessor.processFileTest(file, sampleRate, bufferSize);
        file.close();

        // Store the results
        DFTTime.push_back(DFTResult[6]);
        FFTTime.push_back(FFTResult[6]);
        GoertzelTime.push_back(GoertzelResult[6]);

        DFTCorrect.push_back(DFTResult[0]);
        FFTCorrect.push_back(FFTResult[0]);
        GoertzelCorrect.push_back(GoertzelResult[0]);

        DFTIncorrect.push_back(DFTResult[1]);
        FFTIncorrect.push_back(FFTResult[1]);
        GoertzelIncorrect.push_back(GoertzelResult[1]);

        DFTIncorrectFormat.push_back(DFTResult[2]);
        FFTIncorrectFormat.push_back(FFTResult[2]);
        GoertzelIncorrectFormat.push_back(GoertzelResult[2]);

        DFTTotal.push_back(DFTResult[3]);
        FFTTotal.push_back(FFTResult[3]);
        GoertzelTotal.push_back(GoertzelResult[3]);

        DFTCorrectFormat.push_back(DFTResult[4]);
        FFTCorrectFormat.push_back(FFTResult[4]);
        GoertzelCorrectFormat.push_back(GoertzelResult[4]);

        DFTCorrectTotal.push_back(DFTResult[5]);
        FFTCorrectTotal.push_back(FFTResult[5]);
        GoertzelCorrectTotal.push_back(GoertzelResult[5]);



    }

    // Write to files
    for (int i = 0; i < TestfileNames.size(); ++i) {
        DFT_buffer_CalculationSpeed << DFTTime[i] << std::endl;
        FFT_buffer_CalculationSpeed << FFTTime[i] << std::endl;
        Goertzel_buffer_CalculationSpeed << GoertzelTime[i] << std::endl;
    }
    for (int i = 0; i < TestfileNames.size(); ++i) {
        DFT_correctAndFailFile << DFTCorrect[i] << " " << DFTIncorrect[i] << " " << DFTIncorrectFormat[i] << " " << DFTTotal[i] << " " << DFTCorrectFormat[i] << " " << DFTCorrectTotal[i] << std::endl;
        FFT_correctAndFailFile << FFTCorrect[i] << " " << FFTIncorrect[i] << " " << FFTIncorrectFormat[i] << " " << FFTTotal[i] << " " << FFTCorrectFormat[i] << " " << FFTCorrectTotal[i] << std::endl;
        Goertzel_correctAndFailFile << GoertzelCorrect[i] << " " << GoertzelIncorrect[i] << " " << GoertzelIncorrectFormat[i] << " " << GoertzelTotal[i] << " " << GoertzelCorrectFormat[i] << " " << GoertzelCorrectTotal[i] << std::endl;
    }

    DFT_buffer_CalculationSpeed.close();
    FFT_buffer_CalculationSpeed.close();
    Goertzel_buffer_CalculationSpeed.close();

    DFT_correctAndFailFile.close();
    FFT_correctAndFailFile.close();
    Goertzel_correctAndFailFile.close();

    return 0;

}
#endif // ALGORITHMCOMPARETEST_H
