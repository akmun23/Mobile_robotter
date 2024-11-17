#include "goertzel.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>


// Constructor
GoertzelTesting::GoertzelTesting() {}


double pi = 3.14159265358979323846;

int MinMagnitude = 200;
bool LetterReceivedCompareProgram = false;
bool startOfMessageReceivedCompareProgram = false;
std::vector<char> ReceivedCompareProgram;
std::chrono::high_resolution_clock::time_point clockStartMessageCompareProgram;
std::chrono::high_resolution_clock::time_point clockStartToneCompareProgram;
double timeToReadToneCompareProgram = 0.000234*4;  // timeToReadToneCompareProgram is the time it takes to load and calculate the tone
double timeToSendMessageCompareProgram = timeToReadToneCompareProgram*7;

std::chrono::duration<double> elapsedTimeCompareProgram;


double TimePassed(std::chrono::high_resolution_clock::time_point start){
    elapsedTimeCompareProgram = std::chrono::high_resolution_clock::now() - start;

    return elapsedTimeCompareProgram.count();

}

// Function to read DTMF data from file in chunks
std::vector<double> GoertzelTesting::readDTMFDataChunk(std::ifstream& inFile, int bufferSize) {
    std::vector<double> signal;
    double value;
    std::ofstream outputFile;

    outputFile.open("RecordingFromCompareProgramTest.txt", std::ios_base::app); // The file is opened in append mode meaning that the data will be added to the end of the file

    for (int i = 0; i < bufferSize && inFile >> value; ++i) {
        signal.push_back(value);

        outputFile << value << std::endl;
    }
    return signal;
}

// Function to apply Goertzel algorithm
void goertzel(const std::vector<double>& samples, int targetFreq, int sampleRate, double& power) {
    int numSamples = samples.size();
    double k0 = numSamples*targetFreq/sampleRate;

    double omega_I = cos(2*pi*k0/numSamples);
    //double omega_Q = sin(2*pi*k0/FRAMES_PER_BUFFER);
    double v = 0;
    double v1 = 0;
    double v2 = 0;

    for (int i = 0; i < numSamples; ++i) {
        v  = 2*omega_I*v1 - v2 + samples[i];
        v2 = v1;
        v1 = v;
    }


    power = v1*v1 + v2*v2 - omega_I*v1*v2;
}

// Function to analyze the data using the GoertzelTesting algorithm
void GoertzelTesting::analyzeDataWithGoertzel(const std::vector<double>& data, int sampleRate) {
    std::vector<int> dtmfTones = { 697, 770, 852, 941, 1209, 1336, 1477, 1633 };
    std::vector<double> magnitudes(dtmfTones.size(), 0.0);

    for (size_t i = 0; i < dtmfTones.size(); ++i) {
        goertzel(data, dtmfTones[i], sampleRate, magnitudes[i]);
    }

    if (analyseGoertzelOutput(magnitudes)) {
        //std::cout << "DTMF tone detected!" << std::endl;
    }
    else {
        //std::cout << "No DTMF tone detected." << std::endl;
    }
}

bool analyseGoertzelOutput(const std::vector<double>& mags){
    std::vector<double> rowMags = {mags[0], mags[1], mags[2], mags[3]};
    std::vector<double> columnMags = {mags[4], mags[5], mags[6], mags[7]};

    int maxRow = 0;
    int maxColumn = 0;

    for (int i = 1; i < 4; ++i) {
        if (rowMags[i] > rowMags[maxRow]) {
            maxRow = i;
        }
        if (columnMags[i] > columnMags[maxColumn]) {
            maxColumn = i;
        }
    }

    if(SaveSignal(rowMags,columnMags,maxRow,maxColumn)){
        return true;
    }else{
        return false;
    }

}

bool SaveSignal(std::vector<double> rowMags, std::vector<double> columnMags, int maxRow, int maxColumn){

    if(rowMags[maxRow] > MinMagnitude && columnMags[maxColumn] > MinMagnitude && !LetterReceivedCompareProgram && ((maxRow == 3 && maxColumn == 0) || startOfMessageReceivedCompareProgram)){
        LetterReceivedCompareProgram = true;
        if(maxRow == 0){
            if(maxColumn == 0){
                ReceivedCompareProgram.push_back('1');
            }else if(maxColumn == 1){
                ReceivedCompareProgram.push_back('2');
            }else if(maxColumn == 2){
                ReceivedCompareProgram.push_back('3');
            }else if(maxColumn == 3){
                ReceivedCompareProgram.push_back('A');
            }
        }else if(maxRow == 1){
            if(maxColumn == 0){
                ReceivedCompareProgram.push_back('4');
            }else if(maxColumn == 1){
                ReceivedCompareProgram.push_back('5');
            }else if(maxColumn == 2){
                ReceivedCompareProgram.push_back('6');
            }else if(maxColumn == 3){
                ReceivedCompareProgram.push_back('B');
            }
        }else if(maxRow == 2){
            if(maxColumn == 0){
                ReceivedCompareProgram.push_back('7');
            }else if(maxColumn == 1){
                ReceivedCompareProgram.push_back('8');
            }else if(maxColumn == 2){
                ReceivedCompareProgram.push_back('9');
            }else if(maxColumn == 3){
                ReceivedCompareProgram.push_back('C');
            }
        }else if(maxRow == 3){
            if(maxColumn == 0){
                if(!startOfMessageReceivedCompareProgram){
                    clockStartMessageCompareProgram = std::chrono::high_resolution_clock::now();
                    clockStartToneCompareProgram = std::chrono::high_resolution_clock::now();
                }
                startOfMessageReceivedCompareProgram = true;
                ReceivedCompareProgram.push_back('*');
            }else if(maxColumn == 1){
                ReceivedCompareProgram.push_back('0');
            }else if(maxColumn == 2){
                ReceivedCompareProgram.push_back('#');
            }else if(maxColumn == 3){
                ReceivedCompareProgram.push_back('D');
            }
        }
        return true;
    }
    else if(LetterReceivedCompareProgram && ((TimePassed(clockStartToneCompareProgram)+timeToReadToneCompareProgram/4) > timeToReadToneCompareProgram)){
        LetterReceivedCompareProgram = false;
        clockStartToneCompareProgram = std::chrono::high_resolution_clock::now();
    }

    return false;
}

// Function to process the file and detect DTMF tones in chunks
void GoertzelTesting::processFile(const std::string& filename, int sampleRate, int bufferSize) {

    int Iteration = 1;
    for (int i = 0; i < Iteration; ++i) {

    std::ifstream inFile(filename);
    if (!inFile) {
        std::cerr << "Unable to open file " << filename << std::endl;
        return;
    }

    double maxDuration = 0;
    double sum = 0;
    int tonecounter = 0;
    std::vector<double> data;
    while (true) {
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        data = readDTMFDataChunk(inFile, bufferSize);
        if (data.empty()) break;
        analyzeDataWithGoertzel(data, sampleRate);



        if((TimePassed(clockStartMessageCompareProgram) > timeToSendMessageCompareProgram)  && (ReceivedCompareProgram.size() < 6) && (ReceivedCompareProgram.size() > 0)){

            if(i == Iteration-1){
                printf("\n Message timed out \n");
                fflush(stdout);
                for (int i = 0; i < ReceivedCompareProgram.size(); ++i) {
                    std::cout << ReceivedCompareProgram[i] << " ";
                }
                std::cout << std::endl;
                std::cout <<"----------------------------------------------" << std::endl;
            }
            ReceivedCompareProgram.clear();
            startOfMessageReceivedCompareProgram = false;
            clockStartMessageCompareProgram = std::chrono::high_resolution_clock::now();

        }else if(ReceivedCompareProgram.size() == 6){
            if((ReceivedCompareProgram[0] == '*' && ReceivedCompareProgram[5] == '#') && (TimePassed(clockStartMessageCompareProgram) < timeToSendMessageCompareProgram)){
                if(i == Iteration-1){
                    printf("\n Message correct format \n");
                    fflush(stdout);
                    for (int i = 0; i < ReceivedCompareProgram.size(); ++i) {
                        std::cout << ReceivedCompareProgram[i] << " ";
                    }
                    std::cout << std::endl;
                    std::cout <<"----------------------------------------------" << std::endl;
                }
            }else{
                if(i == Iteration-1){
                    printf("\n Invalid message \n");
                    fflush(stdout);
                    for (int i = 0; i < ReceivedCompareProgram.size(); ++i) {
                        std::cout << ReceivedCompareProgram[i] << " ";
                    }
                    std::cout << std::endl;
                    std::cout <<"----------------------------------------------" << std::endl;
                }
            }
            ReceivedCompareProgram.clear();
            startOfMessageReceivedCompareProgram = false;
            clockStartMessageCompareProgram = std::chrono::high_resolution_clock::now();
        }

        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        //std::cout << "Time taken for processing chunk: " << elapsed.count() << " seconds." << std::endl;
        if (elapsed.count() > maxDuration) {
            maxDuration = elapsed.count();
        }
        sum += elapsed.count();
        tonecounter++;
        timeToReadToneCompareProgram = (sum/tonecounter)*4;
        timeToSendMessageCompareProgram = timeToReadToneCompareProgram*7;
    }

    std::cout << "Average time taken for processing chunks: " << sum / tonecounter << " seconds." << std::endl;
    std::cout << "Maximum time taken for processing a chunk: " << maxDuration << " seconds." << std::endl;

    inFile.close();
    }
}