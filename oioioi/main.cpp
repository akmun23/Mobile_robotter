#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <iostream>
#include <vector>
#include<unistd.h>
#include "sound.h"

std::vector<double> AmplitudeFading;
int SamplesPerFrame = 6000;
// DTMF tone frequencies for digits 0-9
const int LOW_FREQ[16] =  { 941,  697,  697,  697,  770,  770,  770,  852,  852,  852,  697,  770,  852,  941,  941,  941};
const int HIGH_FREQ[16] = {1336, 1209, 1336, 1477, 1209, 1336, 1477, 1209, 1336, 1477, 1633, 1633, 1633, 1633, 1209, 1477};


void makeAmplitudeFading(){

    double Start = 0;
    double fadeInEnd = SamplesPerFrame/6;
    double fadeOutBegin = SamplesPerFrame/6*5;
    double End = SamplesPerFrame;

    for (int i = Start; i < fadeInEnd; i++) {
        AmplitudeFading.push_back(i/fadeInEnd);
    }
    for (int i = fadeInEnd; i < fadeOutBegin; i++) {
        AmplitudeFading.push_back(1.0);
    }
    for (int i = fadeOutBegin; i < End; i++) {
        AmplitudeFading.push_back((End-i)/(End-fadeOutBegin));
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
    int sleep = 150000;

    for (int i = 0; i < time; i++) {
        samples.push_back(sound::SineWave(i, freq1, amp*AmplitudeFading[i])+sound::SineWave(i, freq2, amp*AmplitudeFading[i]));
    }

    buffer.loadFromSamples(&samples[0], samples.size(), 1, 44100);

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
/*
int main() {
    while (true) {
        // Ask user for input
        std::string sequence;
        std::cout << "Enter a sequence of numbers (0-9) or 'q' to quit: ";
        std::cin >> sequence;

        // Check if the user wants to exit
        if (sequence == "q" || sequence == "Q") {
            std::cout << "Exiting program." << std::endl;
            break;
        }

        // Play the sequence of tones
        playSequence(sequence);
    }

    return 0;
}*/


int main() {

    makeAmplitudeFading();

    std::string sequence = "*1010#";  // First frame to be send having direction and speed

    sequence += "*1010#";

    sequence += "*1020#";

    sequence += "*2020#";

    sequence += "*2030#";

    sequence += "*3030#";

    playSequence(sequence);         // Send the frame


    return 0;
}





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




