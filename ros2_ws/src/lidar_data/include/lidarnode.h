#ifndef LIDARNODE_H
#define LIDARNODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

class MappingNode : public rclcpp::Node {
private:
    // Variables to store current position and a list of scanned points in the global map
    double _robot_x = 0.0, _robot_y = 0.0, _robot_yaw = 0.0;
    std::vector<double> map_x_points, map_y_points;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscription;

public:
    MappingNode();

    // Callback for LiDAR data
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // Callback for Odometry data
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif // LIDARNODE_H
