#ifndef LIDARNODE_H
#define LIDARNODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include "GUIWindow.h"
#include "dataTypes.h"


class MappingNode : public rclcpp::Node {
private:
    // Variables to store current position and a list of scanned points in the global map
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscription;
    double _robot_x = 0.0, _robot_y = 0.0, _robot_yaw = 0.0;
    std::vector<double> _distance, _angle;
    GUI gui;

    // Variables to store initial position and orientation
    double _initial_x = 0.0;
    double _initial_y = 0.0;
    double _initial_yaw = 0.0;
    bool initial_pose_set = false;
    bool update = false;

public:
    MappingNode();
    
    ~MappingNode();

    // Callback for LiDAR data
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // Callback for Odometry data
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif // LIDARNODE_H
