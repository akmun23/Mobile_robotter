#include "audio.h"

int main() {
    int sampleRate = 48000;
    int bufferSize = 3000;  // Buffer size for reading chunks
    std::string filename = "C:\\Users\\Jonathan\\Desktop\\C++ Projekter\\SoftwareExamen\\Mobile_robotter\\GetInfoFromMic\\DTMF9R.txt";

    Audio audioProcessor;
    audioProcessor.processFile(filename, sampleRate, bufferSize);

    return 0;
}

