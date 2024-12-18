#ifndef SOUND_H
#define SOUND_H
#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <math.h>
#include <chrono>
std::chrono::high_resolution_clock::time_point startSoundProgram = std::chrono::high_resolution_clock::now();

int soundFrequency = 16000;

namespace sound {
double SineWave(double time, double freq, double amp) {
    double result;
    double tpc = soundFrequency / freq; // ticks per cycle
    double cycles = time / tpc;
    double rad = 2 * M_PI * cycles;
    double amplitude = 32767 * amp;
    result = amplitude * sin(rad);
    return result;
}
}



std::vector<double> AmplitudeFading;
int ToneDuration = 100;  // Tone duration in milliseconds
int PauseDuration = 50; // Pause duration in milliseconds
                        // Time to detect tone 200us
int SamplesPerFrame2 = soundFrequency * (ToneDuration + PauseDuration) / 1000;  // Total duration in samples (tone + pause)
int AudioSamplesPerFrame = SamplesPerFrame2;  // No additional delay since tone + pause combined
int AudioPlayRate = soundFrequency;

// Exponential fade constants
const double fadeInRate = 6.0;  // Rate of fade-in (higher value means faster fade-in)
const double fadeOutRate = 6.0; // Rate of fade-out (higher value means faster fade-out)

// DTMF tone frequencies for digits 0-9
const int LOW_FREQ[16] =  { 941,  697,  697,  697,  770,  770,  770,  852,  852,  852,  697,  770,  852,  941,  941,  941};
const int HIGH_FREQ[16] = {1336, 1209, 1336, 1477, 1209, 1336, 1477, 1209, 1336, 1477, 1633, 1633, 1633, 1633, 1209, 1477};


void makeAmplitudeFading(){

    double AudioStart = 0;
    double fadeInEnd = soundFrequency * ToneDuration / 1000 / 6;  // 1/6th of tone duration
    double fadeOutBegin = soundFrequency * ToneDuration / 1000 - fadeInEnd;  // 5/6th of tone duration
    double fadeOutEnd = soundFrequency * ToneDuration / 1000;
    double End = soundFrequency * (ToneDuration + PauseDuration) / 1000;

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

    std::cout << "AmplitudeFading size: " << AmplitudeFading.size() << std::endl;
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

    float amp = 0.48;

    int time = SamplesPerFrame2;
    int sleep = (1000/(AudioPlayRate/SamplesPerFrame2))*1000+10;    // AudioPlayRate/SamplesPerFrame = 7.35 which is times it is sent per sec
        // This 1000 divided by this gives the time in ms it is sent then times 1000 to get mikrosec

    for (int i = 0; i < time; i++) {
        samples.push_back(sound::SineWave(i, freq1, amp*AmplitudeFading[i])+sound::SineWave(i, freq2, amp*AmplitudeFading[i]));
    }

    buffer.loadFromSamples(&samples[0], samples.size(), 1, AudioPlayRate);
    // Calculated elapsed time from start
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

void RunSoundProgram() {

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


    for (int i = 0; i < soundFrequency; i++) {
        samples.push_back(sound::SineWave(i, 200, 0.5));

    }

    buffer.loadFromSamples(&samples[0], samples.size(), 1, AudioPlayRate);

    sf::Sound sound;
    sound.setBuffer(buffer);
    sound.play();
    usleep(1*1000*1000*2);    // Sleep for 2 seconds
    samples.clear();
    int DelayBetweenTones = 1000*500; // 15 ms

    for(int i = 0; i < 10; ++i){

        playSequence("*15C2#");
        playSequence("*17bc#");
        playSequence("*91ad#");
        playSequence("*7462#");
        playSequence("*1379#");


        /*
        playSequence("*4032#");
        usleep(DelayBetweenTones);
        playSequence("*3240#");
        usleep(DelayBetweenTones);
        playSequence("*2632#");
        usleep(DelayBetweenTones);
        playSequence("*3226#");
        usleep(DelayBetweenTones);
        */

    }
}


#endif
