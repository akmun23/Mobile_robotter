#include "goertzel.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>


// Constructor
GoertzelTesting::GoertzelTesting() {}


double pi = 3.14159265358979323846;

int MinMagnitude = 4000;
bool LetterReceivedCompareProgram = false;
bool startOfMessageReceivedCompareProgram = false;
std::vector<char> ReceivedCompareProgram;
std::chrono::high_resolution_clock::time_point clockStartMessageCompareProgram;
std::chrono::high_resolution_clock::time_point clockStartToneCompareProgram;
double timeToReadToneCompareProgram = 0.00022*4;  // timeToReadToneCompareProgram is the time it takes to load and calculate the tone
double timeToSendMessageCompareProgram = timeToReadToneCompareProgram*5;

std::chrono::duration<double> elapsedTimeCompareProgram;

std::chrono::high_resolution_clock::time_point startToneCalculation;
std::chrono::high_resolution_clock::time_point endToneCalculation;
std::chrono::duration<double> elapsedToneCalculation;
double timeSumToneCalculation = 0;
int toneCounter = 0;


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
    for (int i = 0; i < Iteration; ++i) {

        messageCounter = 1;
        std::ifstream inFile(filename);
        if (!inFile) {
           std::cerr << "Unable to open file " << filename << std::endl;
           return;
        }

        std::vector<double> data;
        while (true) {
            std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
            data = readDTMFDataChunk(inFile, bufferSize);
            if (data.size() < 1500){
                break;
            }
            startToneCalculation = std::chrono::high_resolution_clock::now();
            analyzeDataWithGoertzel(data, sampleRate);
            endToneCalculation = std::chrono::high_resolution_clock::now();;
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
                    }
                }
                ReceivedCompareProgram.clear();
                startOfMessageReceivedCompareProgram = false;
                clockStartMessageCompareProgram = std::chrono::high_resolution_clock::now();
            }



            /*
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
            */

        }
        //std::cout << "Average time taken for processing chunks: " << sum / tonecounter << " seconds." << std::endl;
        //std::cout << "Maximum time taken for processing a chunk: " << maxDuration << " seconds." << std::endl;
        inFile.close();
    }
    /*
    outputFileGoertzel << "----------------------------------------------" << std::endl;
    outputFileGoertzel << "Time to read tone: " << timeToReadToneCompareProgram << std::endl;
    outputFileGoertzel << "Time to send message: " << timeToSendMessageCompareProgram << std::endl;
    outputFileGoertzel << "Message count: " << messageCounter << std::endl;
    outputFileGoertzel << "Correct Messages: " << correct << std::endl;
    outputFileGoertzel << "Incorrect Messages: " << incorrect << std::endl;
    outputFileGoertzel << "Average time taken to send message: " << timeSum/messageCounter << std::endl;
    outputFileGoertzel <<"----------------------------------------------" << std::endl;
    outputFileGoertzel << std::endl;
    outputFileGoertzel << std::endl;
    outputFileGoertzel << std::endl;
    outputFileGoertzel << std::endl;*/



    outputFileGoertzel.close();

}


void GoertzelTesting::checkOutputFile(std::string filename, double calculationTime){


    std::ofstream checkedOutputFile;
    std::ifstream fileToBeChecked;
    checkedOutputFile.open("Checked_Output.txt", std::ios_base::trunc);
    checkedOutputFile.close();

    checkedOutputFile.open("Checked_Output.txt", std::ios_base::app);
    fileToBeChecked.open(filename);
    if (!fileToBeChecked) {
        std::cerr << "Unable to open file " << filename << std::endl;
        return;
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
    checkedOutputFile << "Average time taken to calculate Entire sequence: " << calculationTime*1000  << "ms."<< std::endl;
    checkedOutputFile << "Average time taken to calculate Buffer: " << (timeSumToneCalculation/toneCounter)*1000  << "ms." << std::endl;
    checkedOutputFile << "----------------------------------------------" << std::endl;

    fileToBeChecked.close();
    checkedOutputFile.close();
}





void GoertzelTesting::processFileTest(const std::string& filename, int sampleRate, int bufferSize) {

    int maxCorrect = 0;
    std::vector<double> timeAtMaxCorrect;

    while(timeToReadToneCompareProgram < (0.00029*4)){
        int correctCounter = 0;
        int failCounter = 0;
        std::ifstream inFile(filename);
        if (!inFile) {
            std::cerr << "Unable to open file " << filename << std::endl;
            return;
        }

        std::vector<double> data;
        while (true) {
            std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
            data = readDTMFDataChunk(inFile, bufferSize);
            if (data.size() < 1500){
                break;
            }

            analyzeDataWithGoertzel(data, sampleRate);
            if(((TimePassed(clockStartMessageCompareProgram)) > timeToSendMessageCompareProgram)  && (ReceivedCompareProgram.size() < 6) && (ReceivedCompareProgram.size() > 0)){

                ReceivedCompareProgram.clear();
                startOfMessageReceivedCompareProgram = false;
                clockStartMessageCompareProgram = std::chrono::high_resolution_clock::now();

            }else if(ReceivedCompareProgram.size() == 6){
                if((ReceivedCompareProgram[0] == '*' && ReceivedCompareProgram[5] == '#') && (TimePassed(clockStartMessageCompareProgram) < timeToSendMessageCompareProgram)){
                    correctCounter++;
                }else{
                    failCounter++;
                }


                ReceivedCompareProgram.clear();
                startOfMessageReceivedCompareProgram = false;
                clockStartMessageCompareProgram = std::chrono::high_resolution_clock::now();
            }
        }
        if(correctCounter > maxCorrect){
            maxCorrect = correctCounter;
            timeAtMaxCorrect.clear();
            timeAtMaxCorrect.push_back(timeToReadToneCompareProgram);
        }else if(correctCounter == maxCorrect){
            timeAtMaxCorrect.push_back(timeToReadToneCompareProgram);
        }
        /*
            std::cout <<"----------------------------------------------" << std::endl;
            std::cout << "Correct messages: " << correctCounter << std::endl;
            std::cout << "Incorrect messages: " << failCounter << std::endl;
            std::cout <<"----------------------------------------------" << std::endl;
        */

        if(correctCounter == 50 && failCounter == 0){
            break;
        }else{
            timeToReadToneCompareProgram = timeToReadToneCompareProgram + 0.0000005;
            timeToSendMessageCompareProgram = timeToReadToneCompareProgram*5;
        }


        inFile.close();
    }

    std::cout << "Max correct messages: " << maxCorrect << std::endl;
    std::cout << "Time at max correct messages: " << std::endl;
    for (int i = 0; i < timeAtMaxCorrect.size(); ++i) {
        std::cout << timeAtMaxCorrect[i] << std::endl;
    }
    std::ofstream outputFileGoertzel;

    outputFileGoertzel.open("Goertzel_Test_Output.txt", std::ios_base::trunc);
    outputFileGoertzel.close();

    std::chrono::duration<double> CalculationTime;
    for (int i = 0; i < timeAtMaxCorrect.size(); ++i) {

        std::cout << std::endl;
        std::cout <<"----------------------------------------------" << std::endl;
        std::cout << "Time to read tone: " << timeAtMaxCorrect[i] << std::endl;
        timeToReadToneCompareProgram = timeAtMaxCorrect[i];
        timeToSendMessageCompareProgram = timeToReadToneCompareProgram*5;
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
        processFile(filename, sampleRate, bufferSize);
        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        CalculationTime += end - start;
        std::cout <<"----------------------------------------------" << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
    }
    double elapsedTime = CalculationTime.count();
    checkOutputFile("Goertzel_Test_Output.txt", elapsedTime/timeAtMaxCorrect.size());

}

