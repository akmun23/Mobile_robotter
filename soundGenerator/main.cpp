#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <iostream>
#include <vector>
#include<unistd.h>
#include "sound.h"
#include <chrono>

#include <time.h>

std::vector<double> AmplitudeFading;
int Delay = 750;
int SamplesPerFrame = Delay*8;
int AudioSamplesPerFrame = SamplesPerFrame-Delay*2;
int AudioPlayRate = 40000;

// DTMF tone frequencies for digits 0-9
const int LOW_FREQ[16] =  { 941,  697,  697,  697,  770,  770,  770,  852,  852,  852,  697,  770,  852,  941,  941,  941};
const int HIGH_FREQ[16] = {1336, 1209, 1336, 1477, 1209, 1336, 1477, 1209, 1336, 1477, 1633, 1633, 1633, 1633, 1209, 1477};


void makeAmplitudeFading(){

    double AudioStart = 0;
    double fadeInEnd = AudioSamplesPerFrame/6;
    double fadeOutBegin = AudioSamplesPerFrame-fadeInEnd;
    double fadeOutEnd = AudioSamplesPerFrame;
    double End = AudioSamplesPerFrame+Delay;

    for (int i = AudioStart; i < fadeInEnd; i++) {
        AmplitudeFading.push_back(i/fadeInEnd);
    }
    for (int i = fadeInEnd; i < fadeOutBegin; i++) {
        AmplitudeFading.push_back(1.0);
    }
    for (int i = fadeOutBegin; i < fadeOutEnd; i++) {
        AmplitudeFading.push_back((fadeOutEnd-i)/(fadeOutEnd-fadeOutBegin));
    }
    for (int i = fadeOutEnd; i < End; i++) {
        AmplitudeFading.push_back(0);
    }
}

// Map characters '0'-'9', 'A'-'D', '*', and '#' to corresponding index values
int mapCharToIndex(char key) {

    if(key == '*'){
        key = 'e';
    }
    if(key == '#'){
        key = 'f';
    }

    if (key >= '0' && key <= '9') {
        return key - '0';
    } else if (key >= 'a' && key <= 'f') {
        return 10 + (key - 'a');
    } else {
        return -1; // Invalid key
    }
}

void playTone(double freq1, double freq2){
    sf::SoundBuffer buffer;
    std::vector<sf::Int16> samples;

    float amp = 0.5;

    int time = SamplesPerFrame;
    int sleep = (1000/(AudioPlayRate/SamplesPerFrame))*1000;    // AudioPlayRate/SamplesPerFrame = 7.35 which is times it is sent per sec
                                                                // This 1000 divided by this gives the time in ms it is sent then times 1000 to get mikrosec

    for (int i = 0; i < time; i++) {
        samples.push_back(sound::SineWave(i, freq1, amp*AmplitudeFading[i])+sound::SineWave(i, freq2, amp*AmplitudeFading[i]));
    }

    buffer.loadFromSamples(&samples[0], samples.size(), 1, AudioPlayRate);

    sf::Sound sound;
    sound.setBuffer(buffer);
    sound.play();
    usleep(sleep);
    samples.clear();
}

void playSequence(const std::string &sequence) {
    for (char key : sequence) {
        // Convert the character to the corresponding index
        int index = mapCharToIndex(tolower(key));

        if (index != -1) { // Valid DTMF key
            double freq1 = LOW_FREQ[index];
            double freq2 = HIGH_FREQ[index];
            playTone(freq1, freq2);
        } else {
            std::cout << "Invalid key: " << key << std::endl;
        }
    }
}

int main() {

    makeAmplitudeFading();
    std::string sequenceTone = "*";  // First frame to be send having direction and speed
    std::string sequenceMessage = "*1010#";  // First frame to be send having direction and speed

    /*
    sequence += "*1010#";

    sequence += "*1020#";

    sequence += "*2020#";

    sequence += "*2030#";

    sequence += "*3030#";
    */


    sf::SoundBuffer buffer;
    std::vector<sf::Int16> samples;


    for (int i = 0; i < 44100*2; i++) {
        samples.push_back(sound::SineWave(i, 200, 0.5));

    }

    buffer.loadFromSamples(&samples[0], samples.size(), 1, AudioPlayRate);

    sf::Sound sound;
    sound.setBuffer(buffer);
    sound.play();
    usleep(1*1000*1000*2);    // Sleep for 5
    samples.clear();



    std::vector<double> Data;
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    double sum = 0;
    double min = 0;
    double max = 0;
    /*

    for (int i = 0; i < 2000; ++i) {
        start = std::chrono::high_resolution_clock::now();
        playSequence(sequenceTone);         // Send the frame
        end = std::chrono::high_resolution_clock::now();
        elapsed = end - start;
        Data.push_back(elapsed.count());
    }
    sum = 0;
    min = Data[1];
    max = Data[1];

    for (int i = 1; i < Data.size(); ++i) {
        sum += Data[i];

        if(Data[i] < min){
            min = Data[i];
        }
        if(Data[i] > max){
            max = Data[i];
        }

    }
    std::cout << "--------------------------------------------------------------"<< std::endl;
    std::cout << "Average time for tone and message with audio buffer size 5250"<< std::endl;
    std::cout << "Average time Tone: " << sum/(Data.size()-1) << " seconds" << std::endl;
    std::cout << "Min time Tone: " << min << " seconds" << std::endl;
    std::cout << "Max time Tone: " << max << " seconds" << std::endl;
    std::cout << "First Tone time: " << Data[0] << " seconds" << std::endl;
    std::cout << std::endl;

    Data.clear();

    usleep(1*1000*1000*10);    // Sleep for 10
    */
    for (int i = 1; i < 3; ++i) {
        start = std::chrono::high_resolution_clock::now();
        playSequence(sequenceMessage);         // Send the frame
        end = std::chrono::high_resolution_clock::now();
        elapsed = end - start;
        Data.push_back(elapsed.count());
    }

    sum = 0;
    min = Data[1];
    max = Data[1];
    for (int i = 1; i < Data.size(); ++i) {
        sum += Data[i];

        if(Data[i] < min){
            min = Data[i];
        }
        if(Data[i] > max){
            max = Data[i];
        }
    }
    std::cout << "Average time Message: " << sum/(Data.size()-1) << " seconds" << std::endl;
    std::cout << "Min time Message: " << min << " seconds" << std::endl;
    std::cout << "Max time Message: " << max << " seconds" << std::endl;
    std::cout << "First Message time: " << Data[0] << " seconds" << std::endl;



    return 0;
}
