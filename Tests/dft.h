#include <algorithm>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <complex>
#include <string>

// Function to compute the DFT
void computeDFT(const std::vector<double>& input, std::vector<std::complex<double>>& output);

// Function to find the dominant frequencies
std::vector<double> findDominantFrequency(const std::vector<std::complex<double>>& dft, double samplingRate);

// Function to read DTMF data from file
std::vector<double> readDTMFData(const std::string& filename);

