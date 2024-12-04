#include "dft.h"
#include <unistd.h>

DFT::DFT(int minMagnitude, double timeToReadTone) : MagnitudeAnalysis(minMagnitude, timeToReadTone) {};



std::vector<double> DFT::readDTMFDataChunk(std::ifstream& inFile, int& bufferSize) { // DOESN'T STOP AT END OF FILE
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

// Function to compute the DFT
void DFT::computeDFT(const std::vector<double>& input, int& sampleRate) {
    _TimeForChunkStartDFT = std::chrono::high_resolution_clock::now();

    int N = input.size();
    std::vector<double> output(_tones.size());
    for (int i = 0; i < _tones.size(); ++i) {
        double k = _tones[i]*N/sampleRate;
        std::complex<double> sum = 0.0;
        for (int n = 0; n < N; ++n) {
            double angle = 2 * PI_M * k * n / N;
            sum += input[n] * std::exp(-std::complex<double>(0.0, angle));
        }
        output[i] = std::abs(sum);
    }



    double TimeForCalculationDFT = timePassed(_TimeForChunkStartDFT);
    _TimeSumCalculationDFT += TimeForCalculationDFT;

    if(_calcTimeMaxDFT < TimeForCalculationDFT){
        _calcTimeMaxDFT = TimeForCalculationDFT;
    }
    if(_calcTimeMinDFT == 0 || _calcTimeMinDFT > TimeForCalculationDFT){
        _calcTimeMinDFT = TimeForCalculationDFT;
    }
    usleep(200000); // Sleep for 200 ms

    analyseMagnitudes(output);
}


std::vector<double> DFT::runDFT(std::ifstream &file, int &sampleRate, int &bufferSize){
    _TimeForEntireSequenceStartDFT = std::chrono::high_resolution_clock::now();

    std::ofstream outputFileDFT;
    outputFileDFT.open("DFT_Test_Output.txt", std::ios_base::trunc); // The file is opened in append mode meaning that the data will be added to the end of the file
    outputFileDFT.close();
    outputFileDFT.open("DFT_Test_Output.txt", std::ios_base::app); // The file is opened in append mode meaning that the data will be added to the end of the file
    outputFileDFT << "New sequence of messages" << std::endl;
    std::vector<double> data;


    while (true) {
        data = readDTMFDataChunk(file, bufferSize);
        if (data.size() < bufferSize){
            break;
        }


        computeDFT(data, sampleRate); // computes the DFT calculation on realChunkData and returns it to dftResult

        _countDFT++;

        checkMessageState(outputFileDFT, _correctMessages, _incorrectMessages, _TimeSumCalculationDFT, _messageCounter);



    }
    /*
    std::cout << "The average time for Chunk processing is " << (TimeSumChunkDFT/countDFT)*1000 << " ms." << std::endl;
    std::cout << "The average time for Calculation processing is " << (TimeSumCalculationDFT/countDFT)*1000 << " ms." << std::endl;
    std::cout << "The number of correct messages is: " << correctMessages << std::endl;
    std::cout << "The number of incorrect messages is: " << incorrectMessages << std::endl;
    std::cout << "The number of timed out messages is: " << timedOutMessages << std::endl;
    std::cout << "The % of correct messages is: " << (100*correctMessages)/(correctMessages+incorrectMessages+timedOutMessages) << "%" <<std::endl;
    */

    outputFileDFT.close();
    double calculationTime = timePassed(_TimeForEntireSequenceStartDFT);
    double avgCalcTime = _TimeSumCalculationDFT/_countDFT;
    return checkOutputFile("DFT_Test_Output.txt", calculationTime, "Checked_Output_DFT.txt", avgCalcTime, _calcTimeMaxDFT, _calcTimeMinDFT);
}


std::vector<double> DFT::readDTMFDataDFT(std::ifstream& file, int &sampleRate) {
    std::vector<double> signal;
    double x;

    while (file >> x) {
        signal.push_back(x);
    }

    return signal;
}
