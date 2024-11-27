#include "goertzel.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <unistd.h>
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
std::chrono::high_resolution_clock::time_point clockStartToneCompareProgram1;
std::chrono::high_resolution_clock::time_point clockStartToneCompareProgram2;
std::chrono::high_resolution_clock::time_point clockStartToneCompareProgram3;
std::chrono::high_resolution_clock::time_point clockStartToneCompareProgram4;
std::chrono::high_resolution_clock::time_point clockStartToneCompareProgram5;
std::chrono::high_resolution_clock::time_point clockStartToneCompareProgram6;



double delayBetweenCalculation = 200*1000; // 200 ms in microseconds
double timeToReadToneCompareProgram = 0.201*4;  // timeToReadToneCompareProgram is the time it takes to load and calculate the tone
double timeToSendMessageCompareProgram = timeToReadToneCompareProgram*6.04;

std::chrono::duration<double> elapsedTimeCompareProgram;

std::chrono::high_resolution_clock::time_point startToneCalculation;
std::chrono::high_resolution_clock::time_point endToneCalculation;
std::chrono::duration<double> elapsedToneCalculation;
double timeSumToneCalculation = 0;
int toneCounter = 0;
int NUM_ChannelsCompareProgram = 1;


// TESTING
int MagnitudeCounter = 0;
double rowMagnitudeSum = 0;
double rowMaxMagnitude = 0;
double rowMinMagnitude = 0;
double colMagnitudeSum = 0;
double colMaxMagnitude = 0;
double colMinMagnitude = 0;
int letterCounter = 0;
double ExtraTimeValue = 0;



double TimePassed(std::chrono::high_resolution_clock::time_point start){
    elapsedTimeCompareProgram = std::chrono::high_resolution_clock::now() - start;

    return elapsedTimeCompareProgram.count();

}

// Function to read DTMF data from file in chunks
std::vector<double> GoertzelTesting::readDTMFDataChunk(std::ifstream& inFile, int bufferSize) {
    std::vector<double> signal;
    double value;
    for (int i = 0; i < bufferSize && inFile >> value; ++i) {
        signal.push_back(value);
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
        v  = 2*omega_I*v1 - v2 + samples[i * NUM_ChannelsCompareProgram];
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
        MagnitudeCounter++;
        rowMagnitudeSum += rowMags[maxRow];
        colMagnitudeSum += columnMags[maxColumn];
        if(rowMags[maxRow] > rowMaxMagnitude){
            rowMaxMagnitude = rowMags[maxRow];
        }
        if(rowMags[maxRow] < rowMinMagnitude || MagnitudeCounter == 1){
            rowMinMagnitude = rowMags[maxRow];
        }
        if(columnMags[maxColumn] > colMaxMagnitude){
            colMaxMagnitude = columnMags[maxColumn];
        }
        if(columnMags[maxColumn] < colMinMagnitude || MagnitudeCounter == 1){
            colMinMagnitude = columnMags[maxColumn];
        }
        letterCounter++;
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
                    //clockStartToneCompareProgram = std::chrono::high_resolution_clock::now();
                    clockStartToneCompareProgram1 = std::chrono::high_resolution_clock::now();
                    clockStartToneCompareProgram2 = std::chrono::high_resolution_clock::now();
                    clockStartToneCompareProgram3 = std::chrono::high_resolution_clock::now();
                    clockStartToneCompareProgram4 = std::chrono::high_resolution_clock::now();
                    clockStartToneCompareProgram5 = std::chrono::high_resolution_clock::now();
                    clockStartToneCompareProgram6 = std::chrono::high_resolution_clock::now();
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
    /*else if(LetterReceivedCompareProgram && ((TimePassed(clockStartToneCompareProgram)) > timeToReadToneCompareProgram)){
        LetterReceivedCompareProgram = false;
        clockStartToneCompareProgram = std::chrono::high_resolution_clock::now();
    }*/
    else if(letterCounter == 1 && ((TimePassed(clockStartToneCompareProgram1)+ExtraTimeValue) > timeToReadToneCompareProgram)){
        LetterReceivedCompareProgram = false;
    }else if(letterCounter == 2 && ((TimePassed(clockStartToneCompareProgram2)+ExtraTimeValue) > timeToReadToneCompareProgram*2)){
        LetterReceivedCompareProgram = false;
    }else if(letterCounter == 3 && ((TimePassed(clockStartToneCompareProgram3)+ExtraTimeValue) > timeToReadToneCompareProgram*3)){
        LetterReceivedCompareProgram = false;
    }else if(letterCounter == 4 && ((TimePassed(clockStartToneCompareProgram4)+ExtraTimeValue) > timeToReadToneCompareProgram*4)){
        LetterReceivedCompareProgram = false;
    }else if(letterCounter == 5 && ((TimePassed(clockStartToneCompareProgram5)+ExtraTimeValue) > timeToReadToneCompareProgram*5)){
        LetterReceivedCompareProgram = false;
    }else if(letterCounter == 6 && ((TimePassed(clockStartToneCompareProgram6)+ExtraTimeValue) > timeToReadToneCompareProgram*6)){
        LetterReceivedCompareProgram = false;
        letterCounter = 0;
    }

    return false;
}

// Function to process the file and detect DTMF tones in chunks
void GoertzelTesting::processFile(std::ifstream &file, int sampleRate, int bufferSize) {

    int Iteration = 1;
    /*
    double maxDuration = 0;
    double sum = 0;
    int tonecounter = 0;
    */
    std::ofstream outputFileGoertzel;

    // Open the file and clear it so it is empty

    outputFileGoertzel.open("Goertzel_Test_Output.txt", std::ios_base::app); // The file is opened in append mode meaning that the data will be added to the end of the file
    outputFileGoertzel << "New sequence of messages" << std::endl;
    int correct = 0;
    int incorrect = 0;
    int messageCounter = 1;
    double timeSum = 0;
    file.clear();
    file.seekg(0, std::ios::beg);
    for (int i = 0; i < Iteration; ++i) {

        messageCounter = 1;

        std::vector<double> data;
        while (true) {
            std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
            std::chrono::high_resolution_clock::time_point test = std::chrono::high_resolution_clock::now();
            data = readDTMFDataChunk(file, bufferSize);
            if (data.size() < 1500){
                break;
            }
            startToneCalculation = std::chrono::high_resolution_clock::now();
            analyzeDataWithGoertzel(data, sampleRate);
            endToneCalculation = std::chrono::high_resolution_clock::now();;
            usleep(delayBetweenCalculation);
            elapsedToneCalculation = endToneCalculation - startToneCalculation;
            timeSumToneCalculation += elapsedToneCalculation.count();
            toneCounter++;
            if((TimePassed(clockStartMessageCompareProgram) > timeToSendMessageCompareProgram)  && (ReceivedCompareProgram.size() < 6) && (ReceivedCompareProgram.size() > 0)){

                if(i == Iteration-1){
                    incorrect++;
                    for (int ii = 0; ii < ReceivedCompareProgram.size(); ++ii) {
                        outputFileGoertzel << ReceivedCompareProgram[ii] << " ";
                    }
                    timeSum += TimePassed(clockStartMessageCompareProgram);
                    outputFileGoertzel << std::endl;
                    messageCounter++;
                }
                letterCounter = 0;
                LetterReceivedCompareProgram = false;
                ReceivedCompareProgram.clear();
                startOfMessageReceivedCompareProgram = false;
                clockStartMessageCompareProgram = std::chrono::high_resolution_clock::now();

            }else if(ReceivedCompareProgram.size() == 6){
                if((ReceivedCompareProgram[0] == '*' && ReceivedCompareProgram[5] == '#') && (TimePassed(clockStartMessageCompareProgram) < timeToSendMessageCompareProgram)){
                    if(i == Iteration-1){
                        correct++;
                        for (int ii = 0; ii < ReceivedCompareProgram.size(); ++ii) {
                            outputFileGoertzel << ReceivedCompareProgram[ii] << " ";
                        }
                        timeSum += TimePassed(clockStartMessageCompareProgram);
                        outputFileGoertzel << std::endl;
                        messageCounter++;
                        letterCounter = 0;
                        LetterReceivedCompareProgram = false;

                    }
                }else{
                    if(i == Iteration-1){
                        incorrect++;
                        for (int ii = 0; ii < ReceivedCompareProgram.size(); ++ii) {
                            outputFileGoertzel << ReceivedCompareProgram[ii] << " ";
                        }
                        timeSum += TimePassed(clockStartMessageCompareProgram);
                        outputFileGoertzel << std::endl;
                        messageCounter++;
                        letterCounter = 0;
                        LetterReceivedCompareProgram = false;
                    }
                }
                ReceivedCompareProgram.clear();
                startOfMessageReceivedCompareProgram = false;
                clockStartMessageCompareProgram = std::chrono::high_resolution_clock::now();
            }
        }
    }
    outputFileGoertzel.close();
}


std::vector<double> GoertzelTesting::checkOutputFile(std::string filename, double calculationTime){


    std::ofstream checkedOutputFile;
    std::ifstream fileToBeChecked;
    checkedOutputFile.open("Checked_Output.txt", std::ios_base::trunc);
    checkedOutputFile.close();

    checkedOutputFile.open("Checked_Output.txt", std::ios_base::app);
    fileToBeChecked.open(filename);
    if (!fileToBeChecked) {
        std::cerr << "Unable to open file " << filename << std::endl;
        return {};
    }
    std::string line;

    int messageCounter = 1;
    int correct = 0;
    int incorrectMessage = 0;
    int incorrectFormat = 0;
    while (std::getline(fileToBeChecked, line)) {
        if(line == "* 1 5 C 2 # "){
            correct++;
            checkedOutputFile << "Message " << messageCounter << std::endl;
            checkedOutputFile << "Message Correct Format and correct message" << std::endl;
            checkedOutputFile << line << std::endl;
            checkedOutputFile <<"----------------------------------------------" << std::endl;
            checkedOutputFile << std::endl;

            messageCounter++;
        }else if(line == "* 1 7 B C # "){
            correct++;
            checkedOutputFile << "Message " << messageCounter << std::endl;
            checkedOutputFile << "Message Correct Format and correct message" << std::endl;
            checkedOutputFile << line << std::endl;
            checkedOutputFile <<"----------------------------------------------" << std::endl;
            checkedOutputFile << std::endl;
            messageCounter++;
        }else if(line == "* 9 1 A D # "){
            correct++;
            checkedOutputFile << "Message " << messageCounter << std::endl;
            checkedOutputFile << "Message Correct Format and correct message" << std::endl;
            checkedOutputFile << line << std::endl;
            checkedOutputFile <<"----------------------------------------------" << std::endl;
            checkedOutputFile << std::endl;
            messageCounter++;
        }else if(line == "* 7 4 6 2 # "){
            correct++;
            checkedOutputFile << "Message " << messageCounter << std::endl;
            checkedOutputFile << "Message Correct Format and correct message" << std::endl;
            checkedOutputFile << line << std::endl;
            checkedOutputFile <<"----------------------------------------------" << std::endl;
            checkedOutputFile << std::endl;
            messageCounter++;
        }else if(line == "* 1 3 7 9 # "){
            correct++;
            checkedOutputFile << "Message " << messageCounter << std::endl;
            checkedOutputFile << "Message Correct Format and correct message" << std::endl;
            checkedOutputFile << line << std::endl;
            checkedOutputFile <<"----------------------------------------------" << std::endl;
            checkedOutputFile << std::endl;
            messageCounter++;
        }else if(line == "New sequence of messages"){
            checkedOutputFile << "----------------------------------------------" << std::endl;
            checkedOutputFile << "New sequence of messages" << std::endl;
            checkedOutputFile << "----------------------------------------------" << std::endl;
            checkedOutputFile << std::endl;
        }else if(line[0] == '*' && line[10] == '#'){
            incorrectMessage++;
            checkedOutputFile << "Message " << messageCounter << std::endl;
            checkedOutputFile << "Message Correct Format but incorrect message" << std::endl;
            checkedOutputFile << line << std::endl;
            checkedOutputFile <<"----------------------------------------------" << std::endl;
            checkedOutputFile << std::endl;
            messageCounter++;
        }else{
            incorrectFormat++;
            checkedOutputFile << "Message " << messageCounter << std::endl;
            checkedOutputFile << "Message Incorrect Format" << std::endl;
            checkedOutputFile << line << std::endl;
            checkedOutputFile <<"----------------------------------------------" << std::endl;
            checkedOutputFile << std::endl;
            messageCounter++;
        }
    }
    checkedOutputFile << std::endl;
    checkedOutputFile << "----------------------------------------------" << std::endl;
    checkedOutputFile << "Correct Messages: " << correct << std::endl;
    checkedOutputFile << "Incorrect Messages: " << incorrectMessage << std::endl;
    checkedOutputFile << "Incorrect Format Messages: " << incorrectFormat << std::endl;
    checkedOutputFile << "Total Messages: " << (messageCounter-1) << std::endl;
    checkedOutputFile << "Correct format percentage: " << ((correct+incorrectMessage)*100)/(messageCounter-1) << "%" << std::endl;
    checkedOutputFile << "Correct Messages percentage: " << (correct*100)/(messageCounter-1) << "%" << std::endl;
    checkedOutputFile << "Time taken to calculate Entire sequence: " << calculationTime*1000  << " ms."<< std::endl;
    checkedOutputFile << "Average time taken to calculate Buffer: " << (timeSumToneCalculation/toneCounter)*1000  << " ms." << std::endl;
    checkedOutputFile << "----------------------------------------------" << std::endl;

    fileToBeChecked.close();
    checkedOutputFile.close();

    std::vector<double> outputData;
    outputData.push_back(correct);
    outputData.push_back(incorrectMessage);
    outputData.push_back(incorrectFormat);
    outputData.push_back(messageCounter-1);
    outputData.push_back((correct+incorrectMessage)*100/(messageCounter-1));
    outputData.push_back((correct)*100/(messageCounter-1));
    outputData.push_back((timeSumToneCalculation/toneCounter)*1000);
    return outputData;
}





std::vector<double> GoertzelTesting::processFileTest(std::ifstream &file, int sampleRate, int bufferSize, int NumberOfChannels) {

    int maxCorrect = 0;
    std::vector<double> timeAtMaxCorrect;
    NUM_ChannelsCompareProgram = NumberOfChannels;
    std::ofstream outputFileGoertzel;

    outputFileGoertzel.open("Goertzel_Test_Output.txt", std::ios_base::trunc);
    outputFileGoertzel.close();

    std::chrono::duration<double> CalculationTime;
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
    processFile(file, sampleRate, bufferSize);
    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    CalculationTime += end - start;
    double elapsedTime = CalculationTime.count();
    /*
    std::cout << "Average row magnitude: " << rowMagnitudeSum/MagnitudeCounter << std::endl;
    std::cout << "Average column magnitude: " << colMagnitudeSum/MagnitudeCounter << std::endl;
    std::cout << "Max row magnitude: " << rowMaxMagnitude << std::endl;
    std::cout << "Min row magnitude: " << rowMinMagnitude << std::endl;
    std::cout << "Max column magnitude: " << colMaxMagnitude << std::endl;
    std::cout << "Min column magnitude: " << colMinMagnitude << std::endl;
    */
    return checkOutputFile("Goertzel_Test_Output.txt", elapsedTime);


}

