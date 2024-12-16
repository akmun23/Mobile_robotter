#ifndef CONTROLLERINPUT_H
#define CONTROLLERINPUT_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <math.h>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlError>
#include <QDebug>

class controllerInput : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joySubscriber;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscription;

    bool _initial_pose_set = false;
    double _initial_x = 0;
    double _initial_y = 0;
    double _initial_yaw = 0;
    double _cos_initial = 0;
    double _sin_initial = 0;

    sensor_msgs::msg::LaserScan::SharedPtr _latest_scan;
    nav_msgs::msg::Odometry::SharedPtr _latest_odom;

    // Member variables to store the lates    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;t joystick values
    uint8_t _latestLinearValue = 0;
    uint8_t _latestAngularValue = 0;

    // Previous values to detect changes
    uint8_t _previousLinearValue = 0;
    uint8_t _previousAngularValue = 0;

    // Boolean flag to check if the joystick is active
    bool _joystickActive = false;

    int soundFrequency = 16000;
    std::vector<double> AmplitudeFading;
    int ToneDuration = 100;  // Tone duration in milliseconds
    int PauseDuration = 50; // Pause duration in milliseconds
        // Time to detect tone 200us
    int SamplesPerFrame = soundFrequency * (ToneDuration + PauseDuration) / 1000;  // Total duration in samples (tone + pause)
    int AudioSamplesPerFrame = SamplesPerFrame;  // No additional delay since tone + pause combined
    int AudioPlayRate = soundFrequency;

    // Exponential fade constants
    const double fadeInRate = 6.0;  // Rate of fade-in (higher value means faster fade-in)
    const double fadeOutRate = 6.0; // Rate of fade-out (higher value means faster fade-out)

    // DTMF tone frequencies for digits 0-9
    const int LOW_FREQ[16] =  { 941,  697,  697,  697,  770,  770,  770,  852,  852,  852,  697,  770,  852,  941,  941,  941};
    const int HIGH_FREQ[16] = {1336, 1209, 1336, 1477, 1209, 1336, 1477, 1209, 1336, 1477, 1633, 1633, 1633, 1633, 1209, 1477};

    QSqlDatabase db;

public:
    controllerInput();

    ~controllerInput();

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    void play_dtmf_if_active();

    void makeAmplitudeFading();

    void playTone(double freq1, double freq2);

    // Function to map joystick axis values [-1.0, 1.0] to [0, 100]
    uint8_t mapAxisToByte(float axis_value);

    // Function to play the DTMF sequence for the joystick input
    void playDTMFSequence(uint8_t linear_value, uint8_t angular_value);

    double SineWave(int time, double freq, double amp);
};

#endif // CONTROLLERINPUT_H
