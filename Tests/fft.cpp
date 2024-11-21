#include "fft.h"
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <iterator>
#include <algorithm>

std::chrono::high_resolution_clock::time_point clockStartMessageFFT;
std::chrono::high_resolution_clock::time_point clockStartToneFFT;
std::vector<char> MessageFFT;
bool LetterReceivedFFT = false;
bool startOfMessageReceivedFFT = false;

double TimeToProcessChunkFFT = 0.00033*4;
double timeToSendMessageFFT = TimeToProcessChunkFFT*7;
std::chrono::high_resolution_clock::time_point TimeForEntireSequenceStartFFT;
std::chrono::high_resolution_clock::time_point TimeForChunkStartFFT;
std::chrono::high_resolution_clock::time_point TimeForCalculationStartFFT;
double TimeSumChunkFFT = 0;
double TimeSumCalculationFFT = 0;
int countFFT = 0;


// Constructor for FFTProcessing class
// Initializes member variables
FFTProcessing::FFTProcessing(int minMagnitude, const std::vector<int>& dtmfRowFrequencies, const std::vector<int>& dtmfColumnFrequencies, int frequencyTolerance)
    : minMagnitude(minMagnitude), dtmfRowFrequencies(dtmfRowFrequencies), dtmfColumnFrequencies(dtmfColumnFrequencies), frequencyTolerance(frequencyTolerance),
    timeToReadTone(0.000234 * 4), timeToSendMessage(0.000234 * 4 * 7), letterReceived(false), startOfMessageReceived(false) {}

// Function to reverse bits for FFT
unsigned int FFTProcessing::bitReverse(unsigned int x, int log2n) {
    int n = 0;
    for (int i = 0; i < log2n; i++) {
        n <<= 1;
        n |= (x & 1);
        x >>= 1;
    }
    return n;
}

// Function to perform FFT
std::vector<std::complex<double>> FFTProcessing::fft(const std::vector<std::complex<double>>& input, int log2n) {
    int n = 1 << log2n;
    std::vector<std::complex<double>> a(n);
    // Rearrange input array elements in bit-reversed order
    for (int i = 0; i < n; ++i) {
        a[bitReverse(i, log2n)] = input[i];
    }
    // FFT computation using the Cooley-Tukey algorithm
    for (int s = 1; s <= log2n; ++s) {
        int m = 1 << s;
        std::complex<double> wm = std::exp(std::complex<double>(0, -2 * PI / m));
        for (int k = 0; k < n; k += m) {
            std::complex<double> w = 1;
            for (int j = 0; j < m / 2; ++j) {
                std::complex<double> t = w * a[k + j + m / 2];
                std::complex<double> u = a[k + j];
                a[k + j] = u + t;
                a[k + j + m / 2] = u - t;
                w *= wm;
            }
        }
    }
    return a;
}

// Function to read DTMF data from file in chunks
std::vector<double> FFTProcessing::readDTMFDataFFT(const std::string& filename, int sampleRate) {
    std::vector<double> signal;
    std::ifstream inFile;


    inFile.open(filename);

    if (!inFile) {
        std::cerr << "Unable to open file datafile.txt";
        exit(1);   // call system to stop
    }
    double x;

    while (inFile >> x) {
        signal.push_back(x);
    }

    return signal;
}

// Function to find dominant frequencies in FFT result
std::vector<double> FFTProcessing::findDominantFrequencies(const std::vector<std::complex<double>>& fftResult, int sampleRate) {
    std::unordered_map<int, double> freqMagMap;

    int N = fftResult.size();
    std::vector<double> dominantFreqs;
    std::vector<double> magnitudes;

    // Define the frequency bounds for DTMF tones
    std::vector<std::pair<double, double>> dtmfBounds = {
        {687, 707}, {760, 780}, {842, 862}, {931, 951},
        {1199, 1219}, {1326, 1346}, {1467, 1487}, {1623, 1643}
    };

    for (int i = 0; i < N / 2; ++i) {
        double frequency = i * sampleRate / N;
        bool withinBounds = false;

        for (auto& bounds : dtmfBounds) {
            if (frequency >= bounds.first && frequency <= bounds.second) {
                withinBounds = true;
                break;
            }
        }

        if (withinBounds) {
            magnitudes.push_back(std::abs(fftResult[i]));
        } else {
            magnitudes.push_back(0); // Set magnitude to zero if frequency is out of range
        }
    }

    // Sort magnitudes and get top two frequencies

    double maxRowMagnitude = 0;
    double maxRowIndex = 0;
    double maxColMagnitude = 0;
    double maxColIndex = 0;

    for (int i = 0; i < magnitudes.size(); ++i) {

        if(i* sampleRate / N < 1000){
            if(magnitudes[i] > maxRowMagnitude && magnitudes[i] > 10){
                maxRowMagnitude = magnitudes[i];
                maxRowIndex = i;
            }
        }else if(i * sampleRate / N > 1150){
            if(magnitudes[i] > maxColMagnitude){
                maxColMagnitude = magnitudes[i];
                maxColIndex = i;
            }
        }
    }

    dominantFreqs.push_back(maxRowIndex * sampleRate / N);
    dominantFreqs.push_back(maxColIndex * sampleRate / N);

    return dominantFreqs;

}

// Function to check if a frequency is approximately a DTMF row or column frequency
bool FFTProcessing::isApproximateDTMFRowOrColumnFrequency(int freq) {
    for (int dtmfFreq : dtmfRowFrequencies) {
        if (std::abs(freq - dtmfFreq) <= frequencyTolerance) {
            return true;
        }
    }
    for (int dtmfFreq : dtmfColumnFrequencies) {
        if (std::abs(freq - dtmfFreq) <= frequencyTolerance) {
            return true;
        }
    }
    return false;
}

// Function to calculate elapsed time
double FFTProcessing::timePassed(std::chrono::high_resolution_clock::time_point start) {
    return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
}

// Function to save detected DTMF signals and manage message state
bool FFTProcessing::saveSignal(std::vector<double> rowMags, std::vector<double> columnMags, int maxRow, int maxColumn) {
    if (rowMags[maxRow] > minMagnitude && columnMags[maxColumn] > minMagnitude && !letterReceived && ((maxRow == 3 && maxColumn == 0) || startOfMessageReceived)) {
        letterReceived = true;
        // Save detected tone based on DTMF frequency mapping
        if (maxRow == 0) {
            if (maxColumn == 0) receivedSignal.push_back('1');
            else if (maxColumn == 1) receivedSignal.push_back('2');
            else if (maxColumn == 2) receivedSignal.push_back('3');
            else if (maxColumn == 3) receivedSignal.push_back('A');
        }
        else if (maxRow == 1) {
            if (maxColumn == 0) receivedSignal.push_back('4');
            else if (maxColumn == 1) receivedSignal.push_back('5');
            else if (maxColumn == 2) receivedSignal.push_back('6');
            else if (maxColumn == 3) receivedSignal.push_back('B');
        }
        else if (maxRow == 2) {
            if (maxColumn == 0) receivedSignal.push_back('7');
            else if (maxColumn == 1) receivedSignal.push_back('8');
            else if (maxColumn == 2) receivedSignal.push_back('9');
            else if (maxColumn == 3) receivedSignal.push_back('C');
        }
        else if (maxRow == 3) {
            if (maxColumn == 0) {
                if (!startOfMessageReceived) {
                    clockStartMessage = std::chrono::high_resolution_clock::now();
                    clockStartTone = std::chrono::high_resolution_clock::now();
                }
                startOfMessageReceived = true;
                receivedSignal.push_back('*');
            }
            else if (maxColumn == 1) receivedSignal.push_back('0');
            else if (maxColumn == 2) receivedSignal.push_back('#');
            else if (maxColumn == 3) receivedSignal.push_back('D');
        }
        return true;
    }
    else if (letterReceived && ((timePassed(clockStartTone) + timeToReadTone / 4) > timeToReadTone)) {
        letterReceived = false;
        clockStartTone = std::chrono::high_resolution_clock::now();
    }
    return false;
}

void FFTProcessing::displayReceivedSignal() {
    if (!receivedSignal.empty()) {
        std::cout << "_____________\n";
        for (char ch : receivedSignal) {
            std::cout << ch;
        }
        std::cout << "\n_____________\n";
    }
    receivedSignal.clear();
    startOfMessageReceived = false;
    clockStartMessage = std::chrono::high_resolution_clock::now();
}

void FFTProcessing::processFile(const std::string& filename, int sampleRate, int bufferSize) {
    TimeForEntireSequenceStartFFT = std::chrono::high_resolution_clock::now();
    std::string MessageDetected = "";
    // Read DTMF data from file
    int correctMessages = 0;
    int incorrectMessages = 0;
    int timedOutMessages = 0;
    std::vector<double> data = readDTMFDataFFT(filename, sampleRate);
    if (data.empty()) {
        std::cerr << "Error: No data read from file!" << std::endl;
        return;
    }
    std::ofstream outputFileFFT;
    outputFileFFT.open("FFT_Test_Output.txt", std::ios_base::trunc); // The file is opened in append mode meaning that the data will be added to the end of the file
    outputFileFFT.close();
    outputFileFFT.open("FFT_Test_Output.txt", std::ios_base::app); // The file is opened in append mode meaning that the data will be added to the end of the file
    outputFileFFT << "New sequence of messages" << std::endl;
    // Process the data in chunks of size 3000
    int numChunks = (data.size() + bufferSize - 1) / bufferSize; // Round up to cover remaining samples if any
    for (int chunk = 0; chunk < numChunks; ++chunk) {
        std::chrono::high_resolution_clock::time_point TimeForChunkStartFFT = std::chrono::high_resolution_clock::now();
        int startIndex = chunk * bufferSize;
        int endIndex = std::min(startIndex + bufferSize, static_cast<int>(data.size()));


        // Create a chunk of data to process
        std::vector<std::complex<double>> chunkData(data.begin() + startIndex, data.begin() + endIndex);

        // Zero-padding to make the chunk size equal to the next power of two
        int n = chunkData.size();
        int log2n = std::ceil(std::log2(n));    // Finds the lowest integer value to put to the power of 2 to get n or higher
        int paddedSize = 1 << log2n;            // Finds the 2^log2n value by bit shifting 1 by log2n
        chunkData.resize(paddedSize, { 0.0, 0.0 }); // Zero pad if necessary


        std::vector<std::complex<double>> fftResult(paddedSize);

        // Perform FFT on the chunk
        TimeForCalculationStartFFT = std::chrono::high_resolution_clock::now();
        fftResult = fft(chunkData, log2n);
        TimeSumCalculationFFT += TimePassedFFT(TimeForCalculationStartFFT);


        // Find dominant frequencies in the chunk
        std::vector<double> dominantFrequencies = findDominantFrequencies(fftResult, sampleRate);
        //Process DTMF Tones #NEW
        char detectedTone = getDTMFCharacter(dominantFrequencies[0], dominantFrequencies[1]);


        int state = 5;
        std::pair<int,std::string> MessageAndState = ToneAndMessageHandling(detectedTone, MessageDetected);
        state = MessageAndState.first;
        MessageDetected = MessageAndState.second;

        if(state == 1){
            std::cout << "----------------------------------------"<< std::endl;
            std::cout << "Message received: " << MessageDetected << std::endl;
            correctMessages++;
            outputFileFFT << MessageDetected;
            outputFileFFT << std::endl;
            MessageDetected = "";
        }else if(state == 2){
            std::cout << "----------------------------------------"<< std::endl;
            std::cout << "Message timed out: " << MessageDetected << std::endl;
            timedOutMessages++;
            outputFileFFT << MessageDetected;
            outputFileFFT << std::endl;
            MessageDetected = "";
        }else if(state == 3){
            std::cout << "----------------------------------------"<< std::endl;
            std::cout << "Message wrong format: " << MessageDetected << std::endl;
            incorrectMessages++;
            outputFileFFT << MessageDetected;
            outputFileFFT << std::endl;
            MessageDetected = "";
        }
        TimeSumChunkFFT += TimePassedFFT(TimeForChunkStartFFT);
        TimeSumCalculationFFT += TimePassedFFT(TimeForCalculationStartFFT);
        countFFT++;
    }

    std::cout << "The average time for Chunk processing is " << (TimeSumChunkFFT/countFFT)*1000 << " ms." << std::endl;
    std::cout << "The average time for Calculation processing is " << (TimeSumCalculationFFT/countFFT)*1000 << " ms." << std::endl;
    std::cout << "The number of correct messages is: " << correctMessages << std::endl;
    std::cout << "The number of incorrect messages is: " << incorrectMessages << std::endl;
    std::cout << "The number of timed out messages is: " << timedOutMessages << std::endl;
    std::cout << "The % of correct messages is: " << (100*correctMessages)/(correctMessages+incorrectMessages+timedOutMessages) << "%" <<std::endl;

    outputFileFFT.close();
    double calculationTime = TimePassedFFT(TimeForEntireSequenceStartFFT);
    checkOutputFile("FFT_Test_Output.txt", calculationTime);
}



//Function to get Correct DTMF Char
char FFTProcessing::getDTMFCharacter(double rowFreq, double colFreq){
    {

        if( 687 <= rowFreq && rowFreq <= 707 ){
            if(1199 <= colFreq && colFreq <= 1219){
                return '1';
            }else if(1326 <= colFreq && colFreq <= 1346){
                return '2';
            }else if(1467 <= colFreq && colFreq <= 1487){
                return '3';
            }else if(1623 <= colFreq && colFreq <= 1643){
                return 'A';
            }
        }else if(760 <= rowFreq && rowFreq <= 780){
            if(1199 <= colFreq && colFreq <= 1219){
                return '4';
            }else if(1326 <= colFreq && colFreq <= 1346){
                return '5';
            }else if(1467 <= colFreq && colFreq <= 1487){
                return '6';
            }else if(1623 <= colFreq && colFreq <= 1643){
                return 'B';
            }
        }else if(842 <= rowFreq && rowFreq <= 862){
            if(1199 <= colFreq && colFreq <= 1219){
                return '7';
            }else if(1326 <= colFreq && colFreq <= 1346){
                return '8';
            }else if(1467 <= colFreq && colFreq <= 1487){
                return '9';
            }else if(1623 <= colFreq && colFreq <= 1643){
                return 'C';
            }
        }else if(931 <= rowFreq && rowFreq <= 951){

            if(1199 <= colFreq && colFreq <= 1219){
                if(!startOfMessageReceivedFFT){
                    clockStartMessageFFT = std::chrono::high_resolution_clock::now();
                    clockStartToneFFT = std::chrono::high_resolution_clock::now();
                }
                startOfMessageReceivedFFT = true;
                return '*';
            }else if(1326 <= colFreq && colFreq <= 1346){
                return '0';
            }else if(1467 <= colFreq && colFreq <= 1487){
                return '#';
            }else if(1623 <= colFreq && colFreq <= 1643){
                return 'D';
            }
        }
        return ' ';
    }
}

double FFTProcessing::TimePassedFFT(std::chrono::high_resolution_clock::time_point start){
    std::chrono::duration<double> elapsedTimeMessageFFT;
    elapsedTimeMessageFFT = std::chrono::high_resolution_clock::now() - start;

    return elapsedTimeMessageFFT.count();
}

//Functionto find DTMF message #NEW
// The output int equals state of the message
// 0 - Message not done
// 1 - Message done
// 2 - Message timed out
// 3 - Message wrong format
std::pair<int, std::string> FFTProcessing::ToneAndMessageHandling(char detectedTone, std::string Message){
    if(!LetterReceivedFFT && (detectedTone == '*' || startOfMessageReceivedFFT)){
        LetterReceivedFFT = true;
        if(detectedTone != ' '){
            Message += detectedTone;
        }

        if(((TimePassedFFT(clockStartMessageFFT)) > timeToSendMessageFFT)  && (Message.size() < 6) && (Message.size() > 0)){

            startOfMessageReceivedFFT = false;
            clockStartMessageFFT = std::chrono::high_resolution_clock::now();
            return {2, Message};

        }else if(Message.size() == 6){
            if((Message[0] == '*' && Message[5] == '#') && (TimePassedFFT(clockStartMessageFFT) < timeToSendMessageFFT)){
                startOfMessageReceivedFFT = false;
                clockStartMessageFFT = std::chrono::high_resolution_clock::now();
                return {1, Message};
            }else{
                startOfMessageReceivedFFT = false;
                clockStartMessageFFT = std::chrono::high_resolution_clock::now();
                return {3, Message};
            }
        }
    }else if(LetterReceivedFFT && ((TimePassedFFT(clockStartToneFFT)+TimeToProcessChunkFFT/4) > TimeToProcessChunkFFT)){
        LetterReceivedFFT = false;
        clockStartToneFFT = std::chrono::high_resolution_clock::now();
    }

    return {0, Message};
}




void FFTProcessing::checkOutputFile(std::string filename, double calculationTime){


    std::ofstream checkedOutputFile;
    std::ifstream fileToBeChecked;
    checkedOutputFile.open("Checked_Output_FFT.txt", std::ios_base::trunc);
    checkedOutputFile.close();

    checkedOutputFile.open("Checked_Output_FFT.txt", std::ios_base::app);
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
        if(line == "*15C2#"){
            correct++;
            checkedOutputFile << "Message " << messageCounter << std::endl;
            checkedOutputFile << "Message Correct Format and correct message" << std::endl;
            checkedOutputFile << line << std::endl;
            checkedOutputFile <<"----------------------------------------------" << std::endl;
            checkedOutputFile << std::endl;

            messageCounter++;
        }else if(line == "*17BC#"){
            correct++;
            checkedOutputFile << "Message " << messageCounter << std::endl;
            checkedOutputFile << "Message Correct Format and correct message" << std::endl;
            checkedOutputFile << line << std::endl;
            checkedOutputFile <<"----------------------------------------------" << std::endl;
            checkedOutputFile << std::endl;
            messageCounter++;
        }else if(line == "*91AD#"){
            correct++;
            checkedOutputFile << "Message " << messageCounter << std::endl;
            checkedOutputFile << "Message Correct Format and correct message" << std::endl;
            checkedOutputFile << line << std::endl;
            checkedOutputFile <<"----------------------------------------------" << std::endl;
            checkedOutputFile << std::endl;
            messageCounter++;
        }else if(line == "*7462#"){
            correct++;
            checkedOutputFile << "Message " << messageCounter << std::endl;
            checkedOutputFile << "Message Correct Format and correct message" << std::endl;
            checkedOutputFile << line << std::endl;
            checkedOutputFile <<"----------------------------------------------" << std::endl;
            checkedOutputFile << std::endl;
            messageCounter++;
        }else if(line == "*1379#"){
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
        }else if(line[0] == '*' && line[5] == '#'){
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
    checkedOutputFile << "Time taken to calculate Entire sequence: " << calculationTime * 1000 << " ms."<< std::endl;
    checkedOutputFile << "Average time taken to calculate Buffer: " << (TimeSumCalculationFFT/countFFT)*1000 << " ms." << std::endl;
    checkedOutputFile << "----------------------------------------------" << std::endl;

    fileToBeChecked.close();
    checkedOutputFile.close();
}

