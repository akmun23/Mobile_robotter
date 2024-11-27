#ifndef CONTROLLERINPUT_H
#define CONTROLLERINPUT_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <iostream>
#include <vector>
#include <math.h>

class controllerInput : public rclcpp::Node
{
private:
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joySubscriber;

    // Member variables to store the latest joystick values
    uint8_t _latestLinearValue = 0;
    uint8_t _latestAngularValue = 0;

    // Previous values to detect changes
    uint8_t _previousLinearValue = 0;
    uint8_t _previousAngularValue = 0;

    // Boolean flag to check if the joystick is active
    bool _joystickActive = false;

    std::vector<double> AmplitudeFading;
    int Delay = 1000;
    int SamplesPerFrame = 6*Delay;
    int AudioSamplesPerFrame = SamplesPerFrame-(2*Delay);
    int AudioPlayRate = 44100;

    // DTMF tone frequencies for digits 0-9
    const int LOW_FREQ[16] =  { 941,  697,  697,  697,  770,  770,  770,  852,  852,  852,  697,  770,  852,  941,  941,  941};
    const int HIGH_FREQ[16] = {1336, 1209, 1336, 1477, 1209, 1336, 1477, 1209, 1336, 1477, 1633, 1633, 1633, 1633, 1209, 1477};

public:
    controllerInput();

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void play_dtmf_if_active();

    void makeAmplitudeFading();

    // Map characters '0'-'9', 'A'-'D', '*', and '#' to corresponding index values
    int mapCharToIndex(char key);

    void playTone(double freq1, double freq2);

    void playSequence(const std::string &sequence);

    // Function to map joystick axis values [-1.0, 1.0] to [0, 100]
    uint8_t mapAxisToByte(float axis_value);

    // Function to play the DTMF sequence for the joystick input
    void playDTMFSequence(uint8_t linear_value, uint8_t angular_value);

    double SineWave(int time, double freq, double amp);
};

#endif // CONTROLLERINPUT_H
