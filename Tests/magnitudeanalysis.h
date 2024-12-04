#ifndef MAGNITUDEANALYSIS_H
#define MAGNITUDEANALYSIS_H

#include <fstream>
#include <vector>
#include <chrono>

class MagnitudeAnalysis
{
private:

    // PRIVATE VARIABLES
    std::chrono::high_resolution_clock::time_point _clockStartMessage;
    std::chrono::high_resolution_clock::time_point _clockStartTone1;
    std::chrono::high_resolution_clock::time_point _clockStartTone2;
    std::chrono::high_resolution_clock::time_point _clockStartTone3;
    std::chrono::high_resolution_clock::time_point _clockStartTone4;
    std::chrono::high_resolution_clock::time_point _clockStartTone5;
    std::chrono::high_resolution_clock::time_point _clockStartTone6;

    int _minMagnitude = 0;       // SKAL I CONSTRUCTOR
    int _letterCounter = 0;

    double _timeToReadTone = 0;   // SKAL I CONSTRUCTOR
    double _timeToReadMessage = 0;  // SKAL I CONSTRUCTOR

    bool _letterReceived = false;
    bool _startOfMessageReceived = false;

    std::vector<char> _currentDTMFSequence;
    std::vector<char> _receivedMessage;

public:

    // PUBLIC VARIABLES

    // Constructor
    MagnitudeAnalysis(int magnitude, double timeToReadTone);

    // PUBLIC FUNCTIONS
    void analyseMagnitudes(const std::vector<double>& mags);

    char lookUpDTMFTone(int maxRow, int maxColumn);

    void ActOnDTMFSequence(char FoundTone);

    void ActOnTimeout();

    void SaveSignal(std::vector<double> rowMags, std::vector<double> columnMags, int maxRow, int maxColumn);

    void ResetVariablesAfterMessage();

    double timePassed(std::chrono::high_resolution_clock::time_point& start);

    void checkMessageState(std::ofstream &file, int &correct, int &incorrect, double &timeSum, int &messageCounter);

    std::vector<double> checkOutputFile(std::string filename, double calculationTime, std::string outputFile, double avgCalcTime, double maxCalcTime, double minCalcTime);




    // GETTER FUNCTIONS

    std::vector<char> getReceivedMessage();

    double getStartMessageTimePassed();
    double getStartToneTimePassed(int index);
    double getMessagesTimeToread();
    double getToneTimeToRead();


};

#endif // MAGNITUDEANALYSIS_H
