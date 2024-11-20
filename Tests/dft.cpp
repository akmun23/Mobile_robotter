#include "dft.h"
#include <map>
#include<unordered_map>

std::chrono::high_resolution_clock::time_point clockStartMessageDFT;
std::chrono::high_resolution_clock::time_point clockStartToneDFT;
std::vector<char> MessageDFT;
bool LetterReceivedDFT = false;
bool startOfMessageReceivedDFT = false;

double TimeToProcessChunkDFT = 0.125507*4;
double timeToSendMessageDFT = TimeToProcessChunkDFT*7;
std::chrono::high_resolution_clock::time_point TimeForEntireSequenceStartDFT;
std::chrono::high_resolution_clock::time_point TimeForChunkStartDFT;
std::chrono::high_resolution_clock::time_point TimeForCalculationStartDFT;
double TimeSumChunkDFT = 0;
double TimeSumCalculationDFT = 0;
int countDFT = 0;



struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& pair) const {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

std::vector<double> readDTMFDataDFT(const std::string& filename, int sampleRate) {
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

// Function to compute the DFT
void computeDFT(const std::vector<double>& input, std::vector<std::complex<double>>& output) {
    int N = input.size();
    for (int k = 0; k < N; ++k) {
        std::complex<double> sum = 0.0;
        for (int n = 0; n < N; ++n) {
            double angle = 2 * PI_M * k * n / N;
            sum += input[n] * std::exp(-std::complex<double>(0.0, angle));
        }
        output[k] = sum;
    }
}


// Function to find the dominant frequencies
std::vector<double> findDominantFrequency(const std::vector<std::complex<double>>& dft, double samplingRate) {
    int N = dft.size();
    std::vector<double> dominantFreqs;
    std::vector<double> magnitudes;

    // Define the frequency bounds for DTMF tones
    std::vector<std::pair<double, double>> dtmfBounds = {
        {687, 707}, {760, 780}, {842, 862}, {931, 951},
        {1199, 1219}, {1326, 1346}, {1467, 1487}, {1623, 1643}
    };

    for (int i = 0; i < N / 2; ++i) {
        double frequency = i * samplingRate / N;
        bool withinBounds = false;

        for (auto& bounds : dtmfBounds) {
            if (frequency >= bounds.first && frequency <= bounds.second) {
                withinBounds = true;
                break;
            }
        }

        if (withinBounds) {
            magnitudes.push_back(std::abs(dft[i]));
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

        if(i* samplingRate / N < 1000){
            if(magnitudes[i] > maxRowMagnitude && magnitudes[i] > 10){
                maxRowMagnitude = magnitudes[i];
                maxRowIndex = i;
            }
        }else if(i * samplingRate / N > 1150){
            if(magnitudes[i] > maxColMagnitude){
                maxColMagnitude = magnitudes[i];
                maxColIndex = i;
            }
        }
    }

    dominantFreqs.push_back(maxRowIndex * samplingRate / N);
    dominantFreqs.push_back(maxColIndex * samplingRate / N);

    return dominantFreqs;
}

// Function to read DTMF data from file
std::vector<double> readDTMFData(const std::string& filename) {
    std::vector<double> signal;
    std::ifstream inFile(filename);

    if (!inFile) {
        std::cerr << "Unable to open file " << filename << std::endl;
        exit(1);   // call system to stop
    }

    double x;
    while (inFile >> x) {
        signal.push_back(x);
    }

    return signal;
}

//Funtion to get true DTMF #NEW
char getDTMFCharacter(double rowFreq, double colFreq){

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
            if(!startOfMessageReceivedDFT){
                clockStartMessageDFT = std::chrono::high_resolution_clock::now();
                clockStartToneDFT = std::chrono::high_resolution_clock::now();
            }
            startOfMessageReceivedDFT = true;
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


double TimePassedDFT(std::chrono::high_resolution_clock::time_point start){
    std::chrono::duration<double> elapsedTimeMessageDFT;
    elapsedTimeMessageDFT = std::chrono::high_resolution_clock::now() - start;

    return elapsedTimeMessageDFT.count();

}


//Functionto find DTMF message #NEW
// The output int equals state of the message
// 0 - Message not done
// 1 - Message done
// 2 - Message timed out
// 3 - Message wrong format
std::pair<int, std::string> ToneAndMessageHandling(char detectedTone, std::string Message){
    if(!LetterReceivedDFT && (detectedTone == '*' || startOfMessageReceivedDFT)){
        LetterReceivedDFT = true;
        if(detectedTone != ' '){
            Message += detectedTone;
        }

        if(((TimePassedDFT(clockStartMessageDFT)) > timeToSendMessageDFT)  && (Message.size() < 6) && (Message.size() > 0)){

            startOfMessageReceivedDFT = false;
            clockStartMessageDFT = std::chrono::high_resolution_clock::now();
            return {2, Message};

        }else if(Message.size() == 6){
            if((Message[0] == '*' && Message[5] == '#') && (TimePassedDFT(clockStartMessageDFT) < timeToSendMessageDFT)){
                startOfMessageReceivedDFT = false;
                clockStartMessageDFT = std::chrono::high_resolution_clock::now();
                return {1, Message};
            }else{
                startOfMessageReceivedDFT = false;
                clockStartMessageDFT = std::chrono::high_resolution_clock::now();
                return {3, Message};
            }
        }
    }else if(LetterReceivedDFT && ((TimePassedDFT(clockStartToneDFT)+TimeToProcessChunkDFT/4) > TimeToProcessChunkDFT)){
        LetterReceivedDFT = false;
        clockStartToneDFT = std::chrono::high_resolution_clock::now();
    }

    return {0, Message};
}


void runDFT(std::string filename, int sampleRate, int bufferSize){
    TimeForEntireSequenceStartDFT = std::chrono::high_resolution_clock::now();
    std::string MessageDetected = "";
    // Read DTMF data from file
    int correctMessages = 0;
    int incorrectMessages = 0;
    int timedOutMessages = 0;
    std::vector<double> data = readDTMFDataDFT(filename, sampleRate);
    if (data.empty()) {
        std::cerr << "Error: No data read from file!" << std::endl;
        return;
    }
    std::ofstream outputFileDFT;
    outputFileDFT.open("DFT_Test_Output.txt", std::ios_base::trunc); // The file is opened in append mode meaning that the data will be added to the end of the file
    outputFileDFT.close();
    outputFileDFT.open("DFT_Test_Output.txt", std::ios_base::app); // The file is opened in append mode meaning that the data will be added to the end of the file
    outputFileDFT << "New sequence of messages" << std::endl;
    // Process the data in chunks of size 3000
    int numChunks = (data.size() + bufferSize - 1) / bufferSize; // Round up to cover remaining samples if any
    for (int chunk = 0; chunk < numChunks; ++chunk) {
        TimeForChunkStartDFT = std::chrono::high_resolution_clock::now();
        int startIndex = chunk * bufferSize;
        int endIndex = std::min(startIndex + bufferSize, static_cast<int>(data.size()));


        // Create a chunk of data to process
        std::vector<std::complex<double>> chunkData(data.begin() + startIndex, data.begin() + endIndex);

        // Zero-padding to make the chunk size equal to the next power of two
        int n = chunkData.size();
        int log2n = std::ceil(std::log2(n));    // Finds the lowest integer value to put to the power of 2 to get n or higher
        int paddedSize = 1 << log2n;            // Finds the 2^log2n value by bit shifting 1 by log2n
        chunkData.resize(paddedSize, { 0.0, 0.0 }); // Zero pad if necessary

        /////////////////////////////////////////////// DFT //////////////////////////////////////////////////////////////////


        // Assuming chunkData is of type std::vector<std::complex<double>>
        std::vector<double> realChunkData(chunkData.size());

        // This function extracts the real part of the complex number and stores it in realChunkData
        // This is done by using a lambda function that extracts the real part of the complex number
        // the std::transform function applies the lambda function to each element in chunkData
        std::transform(chunkData.begin(), chunkData.end(), realChunkData.begin(), [](const std::complex<double>& c) {
            return c.real();
        });
        std::vector<std::complex<double>> dftResult(paddedSize); // vector to store the DFT result

        TimeForChunkStartDFT = std::chrono::high_resolution_clock::now();
        computeDFT(realChunkData, dftResult); // computes the DFT calculation on realChunkData and returns it to dftResult
        TimeSumCalculationDFT += TimePassedDFT(TimeForChunkStartDFT);


        // Find dominant frequencies from DFT result
        std::vector<double> DFTdominantFrequencies = findDominantFrequency(dftResult, sampleRate);

        //Process DTMF Tones #NEW
        char detectedTone = getDTMFCharacter(DFTdominantFrequencies[0], DFTdominantFrequencies[1]);

        TimeSumChunkDFT += TimePassedDFT(TimeForChunkStartDFT);
        countDFT++;

        int state = 5;
        std::pair<int,std::string> MessageAndState = ToneAndMessageHandling(detectedTone, MessageDetected);
        state = MessageAndState.first;
        MessageDetected = MessageAndState.second;

        if(state == 1){
            std::cout << "----------------------------------------"<< std::endl;
            std::cout << "Message received: " << MessageDetected << std::endl;
            correctMessages++;
            outputFileDFT << MessageDetected;
            outputFileDFT << std::endl;
            MessageDetected = "";
        }else if(state == 2){
            std::cout << "----------------------------------------"<< std::endl;
            std::cout << "Message timed out: " << MessageDetected << std::endl;
            timedOutMessages++;
            outputFileDFT << MessageDetected;
            outputFileDFT << std::endl;
            MessageDetected = "";
        }else if(state == 3){
            std::cout << "----------------------------------------"<< std::endl;
            std::cout << "Message wrong format: " << MessageDetected << std::endl;
            incorrectMessages++;
            outputFileDFT << MessageDetected;
            outputFileDFT << std::endl;
            MessageDetected = "";
        }

    }

    std::cout << "The average time for Chunk processing is " << (TimeSumChunkDFT/countDFT)*1000 << " ms." << std::endl;
    std::cout << "The average time for Calculation processing is " << (TimeSumCalculationDFT/countDFT)*1000 << " ms." << std::endl;
    std::cout << "The number of correct messages is: " << correctMessages << std::endl;
    std::cout << "The number of incorrect messages is: " << incorrectMessages << std::endl;
    std::cout << "The number of timed out messages is: " << timedOutMessages << std::endl;
    std::cout << "The % of correct messages is: " << (100*correctMessages)/(correctMessages+incorrectMessages+timedOutMessages) << "%" <<std::endl;


    outputFileDFT.close();
    double calculationTime = TimePassedDFT(TimeForEntireSequenceStartDFT);
    checkOutputFile("DFT_Test_Output.txt", calculationTime);
}

void checkOutputFile(std::string filename, double calculationTime){


    std::ofstream checkedOutputFile;
    std::ifstream fileToBeChecked;
    checkedOutputFile.open("Checked_Output_DFT.txt", std::ios_base::trunc);
    checkedOutputFile.close();

    checkedOutputFile.open("Checked_Output_DFT.txt", std::ios_base::app);
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
    checkedOutputFile << "Time taken to calculate Entire sequence: " << calculationTime  << " s."<< std::endl;
    checkedOutputFile << "Average time taken to calculate Buffer: " << (TimeSumCalculationDFT/countDFT)*1000 << " ms." << std::endl;
    checkedOutputFile << "----------------------------------------------" << std::endl;

    fileToBeChecked.close();
    checkedOutputFile.close();
}
