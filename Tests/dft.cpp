#include "dft.h"
#include <unistd.h>

DFT::DFT(double minMagnitude, double timeToReadTone) : MagnitudeAnalysis(minMagnitude, timeToReadTone) {};

DFT::DFT(double minMagnitude, double timeToReadTone, std::string outfile) : MagnitudeAnalysis(minMagnitude, timeToReadTone, outfile) {};

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
    std::chrono::high_resolution_clock::time_point TimeForCalculationStartDFT =std::chrono::high_resolution_clock::now();

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



    double TimeForCalculationDFT = timePassed(TimeForCalculationStartDFT);
    _TimeSumCalculationDFT += TimeForCalculationDFT;
    _AllTimes.push_back(TimeForCalculationDFT);

    if(_calcTimeMaxDFT < TimeForCalculationDFT){
        _calcTimeMaxDFT = TimeForCalculationDFT;
    }
    if(_calcTimeMinDFT == 0 || _calcTimeMinDFT > TimeForCalculationDFT){
        _calcTimeMinDFT = TimeForCalculationDFT;
    }
    usleep(20000); // Sleep for 200 ms

    analyseMagnitudes(output);
}

std::vector<double> DFT::computeDFTCompare(float* input, int sampleRate, int FramesPerBuffer) {
    //std::chrono::high_resolution_clock::time_point TimeForCalculationStartDFT =std::chrono::high_resolution_clock::now();

    int N = FramesPerBuffer;
    std::vector<double> output(_tones.size());
    for (int i = 0; i < _tones.size(); ++i) {
        double k = _tones[i]*N/sampleRate;
        std::complex<double> sum = 0.0;
        for (int n = 0; n < N; ++n) {
            double angle = 2 * PI_M * k * n / N;
            sum += input[n] * std::exp(-std::complex<float>(0.0, angle));
        }
        output[i] = std::abs(sum);
    }
    analyseMagnitudes(output);

    /*
    double TimeForCalculationDFT = timePassed(TimeForCalculationStartDFT);
    _TimeSumCalculationDFT += TimeForCalculationDFT;

    if(_calcTimeMaxDFT < TimeForCalculationDFT){
        _calcTimeMaxDFT = TimeForCalculationDFT;
    }
    if(_calcTimeMinDFT == 0 || _calcTimeMinDFT > TimeForCalculationDFT){
        _calcTimeMinDFT = TimeForCalculationDFT;
    }*/

    return output;
}


std::vector<double> DFT::runDFT(std::ifstream &file, int &sampleRate, int &bufferSize){
    std::chrono::high_resolution_clock::time_point  TimeForEntireSequenceStartDFT = std::chrono::high_resolution_clock::now();

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

        checkMessageState(outputFileDFT, _correctMessages, _incorrectMessages, _messageCounter);



    }


    outputFileDFT.close();
    double calculationTime = timePassed(TimeForEntireSequenceStartDFT);
    double avgCalcTime = _TimeSumCalculationDFT/_countDFT;
    std::cout << "The average time for Calculation processing is " << avgCalcTime*1000<< " ms." << std::endl;
    // sort _AllTimes
    std::sort(_AllTimes.begin(),_AllTimes.end());
    std::cout << "The 25 lowest times are: ";
    for (int i = 0; i < 25; ++i) {
        std::cout << _AllTimes[i]*1000 << " ";
    }
    std::cout << std::endl;
    std::cout << "The 25 lowest times are: ";
    for (int i = _AllTimes.size()-25; i < _AllTimes.size(); ++i) {
        std::cout << _AllTimes[i]*1000 << " ";
    }
    std::cout << std::endl;


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
