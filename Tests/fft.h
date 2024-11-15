#ifndef FFT_H
#define FFT_H

#include <complex>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <iterator>
#include <algorithm>
#include <iostream>

const double PI = 3.1415926536;

// Function to perform bit reversal
unsigned int bitReverse(unsigned int x, int log2n);

// FFT computation function
template<class Iter_T>
void fft(Iter_T a, Iter_T b, int log2n) {
    typedef typename std::iterator_traits<Iter_T>::value_type complex_type;
    const complex_type J(0, 1);
    int n = 1 << log2n;

    for (unsigned int i = 0; i < n; ++i) {
        b[bitReverse(i, log2n)] = a[i];
    }

    for (int s = 1; s <= log2n; ++s) {
        int m = 1 << s;
        int m2 = m >> 1;
        complex_type w(1, 0);
        complex_type wm = exp(-J * (PI / m2));  // Use PI here
        for (int j = 0; j < m2; ++j) {
            for (int k = j; k < n; k += m) {
                complex_type t = w * b[k + m2];
                complex_type u = b[k];
                b[k] = u + t;
                b[k + m2] = u - t;
            }
            w *= wm;
        }
    }
}

// Function to read DTMF data from file
std::vector<std::complex<double>> readDTMFData(const std::string& filename, int& sampleRate);

// Function to find the dominant frequencies in the result of an FFT
std::vector<std::pair<int, double>> findDominantFrequencies(const std::vector<std::complex<double>>& fftResult, int sampleRate);

#endif
