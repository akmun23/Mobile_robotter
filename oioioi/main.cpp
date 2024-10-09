#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <iostream>
#include <vector>
#include<unistd.h>
#include "sound.h"

using namespace std;

// DTMF tone frequencies for digits 0-9
const int LOW_FREQ[16] = {697, 697, 697, 697, 770, 770, 770, 770, 852, 852, 852, 852, 941, 941, 941, 941};
const int HIGH_FREQ[16] = {1209, 1336, 1477, 1633, 1209, 1336, 1477, 1633, 1209, 1336, 1477, 1633, 1209, 1336, 1477, 1633};

// Map characters '0'-'9', 'A'-'D', '*', and '#' to corresponding index values
int mapCharToIndex(char key) {
    key = tolower(key);
    if (key >= '0' && key <= '9') {
        return key - '0';
    } else if (key >= 'a' && key <= 'd') {
        return 10 + (key - 'a');
    } else if (key == 'e') {
        return 12;
    } else if (key == 'f') {
        return 13;
    } else {
        return -1; // Invalid key
    }
}

void playTone(double freq1, double freq2){
    sf::SoundBuffer buffer;
    vector<sf::Int16> samples;

    float amp = 0.5;

    int time = 6000;
    int sleep = 200000;

    for (int i = 0; i < time; i++) {
        samples.push_back(sound::SineWave(i, freq1, amp)+sound::SineWave(i, freq2, amp));
    }

    buffer.loadFromSamples(&samples[0], samples.size(), 1, 44100);

    sf::Sound sound;
    sound.setBuffer(buffer);
    sound.play();
    usleep(sleep);
    samples.clear();
}

void playSequence(const string &sequence) {
    for (char key : sequence) {
        // Convert the character to the corresponding index
        int index = mapCharToIndex(toupper(key));

        if (index != -1) { // Valid DTMF key
            double freq1 = LOW_FREQ[index];
            double freq2 = HIGH_FREQ[index];
            playTone(freq1, freq2);
        } else {
            cout << "Invalid key: " << key << endl;
        }
    }
}

int main() {
    while (true) {
        // Ask user for input
        string sequence;
        cout << "Enter a sequence of numbers (0-9) or 'q' to quit: ";
        cin >> sequence;

        // Check if the user wants to exit
        if (sequence == "q" || sequence == "Q") {
            cout << "Exiting program." << endl;
            break;
        }

        // Play the sequence of tones
        playSequence(sequence);
    }

    return 0;
}
