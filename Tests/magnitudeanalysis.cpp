#include "magnitudeanalysis.h"
#include <algorithm>
#include <iostream>
#include <chrono>



MagnitudeAnalysis::MagnitudeAnalysis(double minMagnitude, double timeToReadTone) : _minMagnitude(minMagnitude), _timeToReadTone(timeToReadTone) {
    _timeToReadMessage = timeToReadTone*6;   // FIND UD AF EN PASSENDE VÆRDI HER ISTEDET FOR 7.2
}


double MagnitudeAnalysis::timePassed(std::chrono::high_resolution_clock::time_point& start){
    std::chrono::duration<double> elapsedTime = std::chrono::high_resolution_clock::now() - start;
    return elapsedTime.count();;
}


void MagnitudeAnalysis::analyseMagnitudes(const std::vector<double>& mags){   // THIS IS THE FUNCTION TO CALL THAT WILL RUN THE WHOLE THING
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
    //std::cout << "Row max: " << maxRow << " Column max: " << maxColumn << std::endl;
    //std::cout << "Row max mags: " << rowMags[maxRow] << " Column max mags: " << columnMags[maxColumn] << std::endl;

    SaveSignal(rowMags,columnMags,maxRow,maxColumn);

}


void MagnitudeAnalysis::SaveSignal(std::vector<double> rowMags, std::vector<double> columnMags, int maxRow, int maxColumn){
    if (rowMags[maxRow] > _minMagnitude && columnMags[maxColumn] > _minMagnitude && !_letterReceived && ((maxRow == 3 && maxColumn == 0) || _startOfMessageReceived)){
        _currentDTMFSequence.push_back(lookUpDTMFTone(maxRow,maxColumn));
    }
    if(_letterCounter == 1 && ((timePassed(_clockStartTone1)) > _timeToReadTone)){
        _currentDTMFSequence.clear();
        _letterReceived = false;
        _letterCounter++;

    }else if(_letterCounter == 2 && ((timePassed(_clockStartTone2)) > _timeToReadTone*2)){
        ActOnTimeout();
        _letterReceived = false;
    }else if(_letterCounter == 3 && ((timePassed(_clockStartTone3)) > _timeToReadTone*3)){
        ActOnTimeout();
        _letterReceived = false;
    }else if(_letterCounter == 4 && ((timePassed(_clockStartTone4)) > _timeToReadTone*4)){
        ActOnTimeout();
        _letterReceived = false;
    }else if(_letterCounter == 5 && ((timePassed(_clockStartTone5)) > _timeToReadTone*5)){
        ActOnTimeout();
        _letterReceived = false;
    }else if(_letterCounter == 6 && ((timePassed(_clockStartTone6)) > _timeToReadTone*6)){
        ActOnTimeout();
        _letterReceived = false;
    }
}



char MagnitudeAnalysis::lookUpDTMFTone(int maxRow, int maxColumn){
    if(maxRow == 0){
        if(maxColumn == 0){
            return('1');
        }else if(maxColumn == 1){
            return('2');
        }else if(maxColumn == 2){
            return('3');
        }else if(maxColumn == 3){
            return('A');
        }
    }else if(maxRow == 1){
        if(maxColumn == 0){
            return('4');
        }else if(maxColumn == 1){
            return('5');
        }else if(maxColumn == 2){
            return('6');
        }else if(maxColumn == 3){
            return('B');
        }
    }else if(maxRow == 2){
        if(maxColumn == 0){
            return('7');
        }else if(maxColumn == 1){
            return('8');
        }else if(maxColumn == 2){
            return('9');
        }else if(maxColumn == 3){
            return('C');
        }
    }else if(maxRow == 3){
        if(maxColumn == 0){
            if(!_startOfMessageReceived){
                _clockStartMessage = std::chrono::high_resolution_clock::now();
                _clockStartTone1 = std::chrono::high_resolution_clock::now();
                _clockStartTone2 = std::chrono::high_resolution_clock::now();
                _clockStartTone3 = std::chrono::high_resolution_clock::now();
                _clockStartTone4 = std::chrono::high_resolution_clock::now();
                _clockStartTone5 = std::chrono::high_resolution_clock::now();
                _clockStartTone6 = std::chrono::high_resolution_clock::now();
                _startOfMessageReceived = true;
                ActOnDTMFSequence('*');
            }
            return('*');
        }else if(maxColumn == 1){
            return('0');
        }else if(maxColumn == 2){
            return('#');
        }else if(maxColumn == 3){
            return('D');
        }
    }
    return ' ';
}
int MagnitudeAnalysis::getValueFromLetter(char letter){

    if(letter == '0'){
        return 0;
    }else if(letter == '1'){
        return 1;
    }else if(letter == '2'){
        return 2;
    }else if(letter == '3'){
        return 3;
    }else if(letter == '4'){
        return 4;
    }else if(letter == '5'){
        return 5;
    }else if(letter == '6'){
        return 6;
    }else if(letter == '7'){
        return 7;
    }else if(letter == '8'){
        return 8;
    }else if(letter == '9'){
        return 9;
    }else if(letter == 'A'){
        return 10;
    }else if(letter == 'B'){
        return 11;
    }else if(letter == 'C'){
        return 12;
    }else if(letter == 'D'){
        return 13;
    }else if(letter == '*'){
        return 14;
    }else if(letter == '#'){
        return 15;
    }
    return 0;
}

void MagnitudeAnalysis::ActOnDTMFSequence(char FoundTone){

    if (FoundTone == ' '){
        std::cout << "No tone detected" << std::endl;
    }else{
        std::cout << "Tone spotted : " << FoundTone << std::endl;
    }
    _receivedMessage.push_back(FoundTone);
    _receivedValues.push_back(getValueFromLetter(FoundTone));
    _startOfMessageReceived = true;
    _letterCounter++;
    _currentDTMFSequence.clear();
}

void MagnitudeAnalysis::ActOnTimeout(){

    if (_currentDTMFSequence.size() > 0){
        std::sort( _currentDTMFSequence.begin(), _currentDTMFSequence.end() );
        //std::cout << "Timeout" << std::endl;
        for (int i = 0; i < _currentDTMFSequence.size(); ++i) {
            std::cout << _currentDTMFSequence[i] << " ";
        }
        char currentChar = _currentDTMFSequence[0];
        char mostChar = _currentDTMFSequence[0];
        int currentCount = 1;
        int mostCount = 1;
        for (int i = 1;  i < _currentDTMFSequence.size(); i++)
        {
            if (_currentDTMFSequence[i] == currentChar)
            {
                currentCount++;
            }else
            {
                currentCount = 1;
            }
            if (currentCount > mostCount)
            {
                mostCount = currentCount;
                mostChar = currentChar;
            }
            currentChar = _currentDTMFSequence[i];
        }

        //std::cout << "Most freq char is '" << mostChar << "', appears " << mostCount << " times.\n";
        ActOnDTMFSequence(mostChar);
    }
    else{
        ActOnDTMFSequence(' ');
    }

}

void MagnitudeAnalysis::ResetVariablesAfterMessage(){
    _letterCounter = 0;
    _currentDTMFSequence.clear();
    _letterReceived = false;
    _receivedMessage.clear();
    _receivedValues.clear();
    _startOfMessageReceived = false;
    _clockStartMessage = std::chrono::high_resolution_clock::now();
    std::cout << std::endl;
    std::cout << "NEW MESSAGE" << std::endl;
    std::cout << std::endl;
}

void MagnitudeAnalysis::reactOnSignal(){

    _drivingSpeed = (_receivedValues[1]*16+_receivedValues[2]);
    _direction = (_receivedValues[3]*16+_receivedValues[4]);
    _fullMessageReceived = true;

}


void MagnitudeAnalysis::checkMessageState(std::ofstream &file, int &correct, int &incorrect, int &messageCounter){

    if((getStartMessageTimePassed() > getMessagesTimeToread())  && (_receivedMessage.size() < 6) && (_receivedMessage.size() > 0)){
        incorrect++;
        for (int ii = 0; ii < _receivedMessage.size(); ++ii) {
            file << _receivedMessage[ii] << " ";
        }
        file << std::endl;
        messageCounter++;
        std::cout << "Message Incorrect Format" << std::endl;
        reactOnSignal();
        ResetVariablesAfterMessage();

    }else if(_receivedMessage.size() == 6){
        if((_receivedMessage[0] == '*' && _receivedMessage[5] == '#') && (getStartMessageTimePassed() < getMessagesTimeToread())){
            correct++;
            reactOnSignal();
            std::cout << "Message Correct Format" << std::endl;

        }else{
            incorrect++;
            reactOnSignal();
            std::cout << "Message Incorrect" << std::endl;

        }


        for (int ii = 0; ii < _receivedMessage.size(); ++ii) {
            file << _receivedMessage[ii] << " ";
        }
        file << std::endl;
        messageCounter++;
        reactOnSignal();
        ResetVariablesAfterMessage();
    }
}

void MagnitudeAnalysis::checkMessageState(){
    if((getStartMessageTimePassed() > getMessagesTimeToread())  && (_receivedMessage.size() < 6) && (_receivedMessage.size() > 0)){

        std::cout << "Message Incorrect Format" << std::endl;
        reactOnSignal();
        ResetVariablesAfterMessage();

    }else if(_receivedMessage.size() == 6){
        if((_receivedMessage[0] == '*' && _receivedMessage[5] == '#') && (getStartMessageTimePassed() < getMessagesTimeToread())){
            reactOnSignal();
            std::cout << "Message Correct Format" << std::endl;

        }else{
            reactOnSignal();
            std::cout << "Message Incorrect" << std::endl;

        }
        reactOnSignal();
        ResetVariablesAfterMessage();
    }
}








std::vector<double> MagnitudeAnalysis::checkOutputFile(std::string filename, double calculationTime, std::string outputFile, double avgCalcTime, double maxCalcTime, double minCalcTime){


    std::ofstream checkedOutputFile;
    std::ifstream fileToBeChecked;
    checkedOutputFile.open(outputFile, std::ios_base::trunc);
    checkedOutputFile.close();
    checkedOutputFile.open(outputFile, std::ios_base::app);
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
    checkedOutputFile << "Average time taken to calculate Buffer: " << avgCalcTime*1000 << " ms." << std::endl;
    checkedOutputFile << "Max time taken to calculate Buffer: " << maxCalcTime * 1000 << " ms." << std::endl;
    checkedOutputFile << "Min time taken to calculate Buffer: " << minCalcTime * 1000 << " ms." << std::endl;
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
    outputData.push_back(avgCalcTime*1000);
    outputData.push_back(maxCalcTime*1000);
    outputData.push_back(minCalcTime*1000);
    return outputData;
}





/////////////////////////////////////////////////////// GETTER FUNCTIONS //////////////////////////////////////////////////////
std::vector<char> MagnitudeAnalysis::getReceivedMessage(){
    return _receivedMessage;
}
std::vector<int> MagnitudeAnalysis::getReceivedValues(){
    return _receivedValues;
}

double MagnitudeAnalysis::getStartMessageTimePassed(){
    return timePassed(_clockStartMessage);
}
double MagnitudeAnalysis::getStartToneTimePassed(int index){
    if(index == 1){
        return timePassed(_clockStartTone1);
    }else if(index == 2){
        return timePassed(_clockStartTone2);
    }else if(index == 3){
        return timePassed(_clockStartTone3);
    }else if(index == 4){
        return timePassed(_clockStartTone4);
    }else if(index == 5){
        return timePassed(_clockStartTone5);
    }else if(index == 6){
        return timePassed(_clockStartTone6);
    }
    return 0.0;
}


double MagnitudeAnalysis::getMessagesTimeToread(){
    return _timeToReadMessage;
}
double MagnitudeAnalysis::getToneTimeToRead(){
    return _timeToReadTone;
}

int MagnitudeAnalysis::getDrivingSpeed(){
    return _drivingSpeed;
}
int MagnitudeAnalysis::getDirection(){
    return _direction;
}

bool MagnitudeAnalysis::getMessagesReceived(){
    return _fullMessageReceived;
}

///////////////////////////////////////////// SETTER FUNCTIONS ////////////////////////////////////////////////////

void MagnitudeAnalysis::setMessagesReceived(bool received){
    _fullMessageReceived = received;
}
