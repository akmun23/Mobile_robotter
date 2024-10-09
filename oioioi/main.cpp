#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <vector>
#include<unistd.h>
#include "sound.h"

using namespace std;

int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "my window");

    sf::SoundBuffer buffer;
    vector<sf::Int16> samples;

    float amp = 0.5;

    int time = 6000;
    int sleep = 200000;

    for (int i = 0; i < time; i++) {
        samples.push_back(sound::SineWave(i, 697, amp)+sound::SineWave(i, 1209, amp));
    }

    buffer.loadFromSamples(&samples[0], samples.size(), 1, 44100);

    sf::Sound sound;
    sound.setBuffer(buffer);
    sound.play();
    usleep(sleep);
    samples.clear();

    for (int i = 0; i < time; i++) {
        samples.push_back(sound::SineWave(i, 770, amp)+sound::SineWave(i, 1209, amp));
    }

    buffer.loadFromSamples(&samples[0], samples.size(), 1, 44100);

    sound.setBuffer(buffer);
    sound.play();
    usleep(sleep);
    samples.clear();


    for (int i = 0; i < time; i++) {
        samples.push_back(sound::SineWave(i, 770, amp)+sound::SineWave(i, 1477, amp));
    }

    buffer.loadFromSamples(&samples[0], samples.size(), 1, 44100);

    sound.setBuffer(buffer);
    sound.play();
    usleep(sleep);
    samples.clear();

    for (int i = 0; i < time; i++) {
        samples.push_back(sound::SineWave(i, 941, amp)+sound::SineWave(i, 1633, amp));
    }

    buffer.loadFromSamples(&samples[0], samples.size(), 1, 44100);

    sound.setBuffer(buffer);
    sound.play();
    usleep(sleep);
    samples.clear();

    while (window.isOpen()) {
        sf::Event event;

        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }
    }

    return 0;
}
