#include "audio.h"
#include "dft.h"
#include "sound.h"

//#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>

void RunListeningProgram() {
    Audio audio;
    audio.Init();
    audio.start();
    audio.end();
}


int main() {

    /*

    std::string filename = "/home/emil/workspace/fourier/DTMF9R.txt"; // Replace with your actual file path
    double samplingRate = 48000; // Example sampling rate in Hz

    // Read DTMF data from file
    std::vector<double> signal = readDTMFData(filename);

    std::vector<std::complex<double>> dft;
    int bufferSize = 3000;

    for (int k = 0; k < (signal.size()) / bufferSize; k++) {
        std::vector<double> temp;
        for (int kk = 0; kk < bufferSize; kk++) {
            temp.push_back(signal[bufferSize * k + kk]);
        }
        computeDFT(temp, dft);
        std::vector<double> dominantFrequency = findDominantFrequency(dft, samplingRate);
        std::cout << "Sample number: " << k << std::endl;
        std::cout << "Dominant Frequency: " << dominantFrequency[0] << " Hz" << std::endl;
        std::cout << "Second Dominant Frequency: " << dominantFrequency[1] << " Hz" << std::endl;
        std::cout << std::endl;
    }*/






    std::thread thread1(RunSoundProgram);
    std::thread thread2(RunListeningProgram);


    /*
    // Setup of audio
    Audio audio;
    audio.Init();
    audio.start();
    audio.end();*/

    thread1.join();
    thread2.join();



    return EXIT_SUCCESS;

}


