#ifndef DFT_H
#define DFT_H

#include <chrono>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <complex>
#include <string>

const double PI_M = 3.1415926536;

// Function to compute the DFT
void computeDFT(const std::vector<double>& input, std::vector<std::complex<double>>& output);

// Function to find the dominant frequencies
std::vector<double> findDominantFrequency(const std::vector<std::complex<double>>& dft, double samplingRate);

// Function to read DTMF data from file
std::vector<double> readDTMFDataDFT(const std::string& filename, int sampleRate);

//Function to get Correct DTMF Char
char getDTMFCharacter(double rowFreq, double colFreq);

double TimePassedDFT(std::chrono::high_resolution_clock::time_point start);

//Function to get DTMF Message
std::pair<int, std::string> ToneAndMessageHandling(char detectedTone, std::string Message);

void runDFT(std::string filename, int sampleRate, int bufferSize);

//Function to get DTMF Message
std::pair<int, std::string> ToneAndMessageHandling(char detectedTone, std::string Message);

void checkOutputFile(std::string filename, double calculationTime);


#endif
