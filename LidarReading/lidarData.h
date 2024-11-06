#ifndef LIDARDATA_H
#define LIDARDATA_H

#include "windowClass.h"
#include <vector>
#include <iostream>
#include <conio.h>
#include <math.h>
#include <QApplication>
#include <QSurfaceFormat>

class MappingNode : public rclcpp::Node {
private:
    // Variables to store current position and a list of scanned points in the global map
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_subscription;
    //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscription;

public:
    MappingNode() : Node("mapping_node");
    
    // Callback for Odometry data
    //void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    // Callback for LiDAR data
    std::vector<std::vector<double>> scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
}

#endif
