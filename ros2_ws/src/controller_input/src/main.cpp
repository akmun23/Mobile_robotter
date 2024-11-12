#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <iostream>
#include <vector>
#include <memory>
#include <unistd.h>
#include <cmath>
#include "sound.h"

// DTMF tone frequencies for digits 0-9
const int LOW_FREQ[16] =  { 941,  697,  697,  697,  770,  770,  770,  852,  852,  852,  697,  770,  852,  941,  941,  941};
const int HIGH_FREQ[16] = {1336, 1209, 1336, 1477, 1209, 1336, 1477, 1209, 1336, 1477, 1633, 1633, 1633, 1633, 1209, 1477};

// Map characters '0'-'9', 'A'-'D', '*', and '#' to corresponding index values
int mapCharToIndex(char key) {
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

    int time = 6000;
    int sleep = 150000;

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

// Function to map joystick axis values [-1.0, 1.0] to [0, 255]
uint8_t mapAxisToByte(float axis_value) {
    return static_cast<uint8_t>(round((axis_value + 1.0) / 2.0 * 255));
}

// Function to play the DTMF sequence for the joystick input
void playDTMFSequence(uint8_t linear_value, uint8_t angular_value) {
    // Play the start tone (e.g., DTMF for '*')
    playTone(941, 1209);

    // Play two tones for the linear velocity
    int linear_tone1 = (linear_value >> 4) & 0x0F;  // Higher nibble (4 bits)
    int linear_tone2 = linear_value & 0x0F;          // Lower nibble (4 bits)
    playTone(LOW_FREQ[linear_tone1], HIGH_FREQ[linear_tone1]);
    playTone(LOW_FREQ[linear_tone2], HIGH_FREQ[linear_tone2]);

    // Play two tones for the angular velocity
    int angular_tone1 = (angular_value >> 4) & 0x0F; // Higher nibble (4 bits)
    int angular_tone2 = angular_value & 0x0F;        // Lower nibble (4 bits)
    playTone(LOW_FREQ[angular_tone1], HIGH_FREQ[angular_tone1]);
    playTone(LOW_FREQ[angular_tone2], HIGH_FREQ[angular_tone2]);

    // Play the stop tone (e.g., DTMF for '#')
    playTone(941, 1477);
}

class ControllerInput : public rclcpp::Node
{
private:
    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joySubscriber;
    rclcpp::TimerBase::SharedPtr _timer;

    // Member variables to store the latest joystick values
    uint8_t _latestLinearValue = 0;
    uint8_t _latestAngularValue = 0;

    // Previous values to detect changes
    uint8_t _previousLinearValue = 0;
    uint8_t _previousAngularValue = 0;

    // Boolean flag to check if the joystick is active
    bool _joystickActive = false;


public:
    ControllerInput() : Node("controller_input")
    {
        // Create a publisher for cmd_vel
        // _publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscribe to the joystick messages
        _joySubscriber = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ControllerInput::joy_callback, this, std::placeholders::_1));

        // Initialize the timer to call the play_dtmf_if_active function every 1 second
        _timer = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ControllerInput::play_dtmf_if_active, this)
            );
    }
    
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Map joystick values to the range [0, 255]
        uint8_t linear_value = mapAxisToByte(msg->axes[1]);  // Linear velocity value
        uint8_t angular_value = mapAxisToByte(msg->axes[0]); // Angular velocity value

        // Update the active status based on whether any joystick axes are non-zero
        _joystickActive = (msg->axes[0] != 0.0f || msg->axes[1] != 0.0f);

        // Save the values to member variables for use in the timer callback
        _latestLinearValue = linear_value;
        _latestAngularValue = angular_value;

		/*
        // Create a Twist message for the robot
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = (static_cast<float>(linear_value) / 255.0) * 2.0 - 1.0;  // Convert back to -1.0 to 1.0
        twist_msg.angular.z = (static_cast<float>(angular_value) / 255.0) * 2.0 - 1.0; // Convert back to -1.0 to 1.0
        _publisher->publish(twist_msg);
        */
    }

    void play_dtmf_if_active() {
        // Check if the joystick values have changed, including changing to zero
        if (_latestLinearValue != _previousLinearValue || _latestAngularValue != _previousAngularValue) {
            // Debug print to show the mapped values
            std::cout << "Playing DTMF for Linear Value: " << static_cast<int>(_latestLinearValue)
                      << ", Angular Value: " << static_cast<int>(_latestAngularValue) << std::endl;

            // Play the DTMF sequence if joystick is active or has changed to zero
            if (_joystickActive || _latestLinearValue != 0 || _latestAngularValue != 0) {
                playDTMFSequence(_latestLinearValue, _latestAngularValue);
            }

            // Update the previous values after playing the sequence
            _previousLinearValue = _latestLinearValue;
            _previousAngularValue = _latestAngularValue;
        }
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerInput>());
    rclcpp::shutdown();
    return 0;
}
