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

    double _minMagnitude = 0;       // SKAL I CONSTRUCTOR
    int _letterCounter = 0;
    std::string _outFileName;

    int _drivingSpeed = 0;
    int _direction = 0;

    double _timeToReadTone = 0;   // SKAL I CONSTRUCTOR
    double _timeToReadMessage = 0;  // SKAL I CONSTRUCTOR

    bool _letterReceived = false;
    bool _startOfMessageReceived = false;
    bool _fullMessageReceived = false;

    std::vector<char> _currentDTMFSequence;
    std::vector<char> _receivedMessage;
    std::vector<int> _receivedValues;

    std::vector<double> _HammingWindow;

public:

    // PUBLIC VARIABLES

    // Constructor
    MagnitudeAnalysis(double magnitude, double timeToReadTone);

    MagnitudeAnalysis(double magnitude, double timeToReadTone, std::string outFileName);

    // PUBLIC FUNCTIONS
    void analyseMagnitudes(const std::vector<double>& mags);

    char lookUpDTMFTone(int maxRow, int maxColumn);

    void ActOnDTMFSequence(char FoundTone);

    void ActOnTimeout();

    void SaveSignal(std::vector<double> rowMags, std::vector<double> columnMags, int maxRow, int maxColumn);

    void ResetVariablesAfterMessage();

    double timePassed(std::chrono::high_resolution_clock::time_point& start);

    void checkMessageState(std::ofstream &file, int &correct, int &incorrect, int &messageCounter);
    void checkMessageState();


    std::vector<double> checkOutputFile(std::string filename, double calculationTime, std::string outputFile, double avgCalcTime, double maxCalcTime, double minCalcTime);

    int getValueFromLetter(char letter);

    void reactOnSignal();


    // GETTER FUNCTIONS

    std::vector<char> getReceivedMessage();
    std::vector<int> getReceivedValues();

    double getStartMessageTimePassed();
    double getStartToneTimePassed(int index);
    double getMessagesTimeToread();
    double getToneTimeToRead();
    int getDrivingSpeed();
    int getDirection();
    bool getMessagesReceived();

    std::vector<double> getHammingWindow();

    // SETTER FUNCTIONS
    void setMessagesReceived(bool received);


};

#endif // MAGNITUDEANALYSIS_H
