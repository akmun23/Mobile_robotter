#ifndef LIDARNODE_H
#define LIDARNODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <mutex>
#include <vector>
#include "GUIWindow.h"
#include "dataTypes.h"

class MappingNode : public rclcpp::Node {
private:
    // Variables to store current position and a list of scanned points in the global map
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_subscription;

    double _robot_x = 0.0, _robot_y = 0.0, _robot_yaw = 0.0;
    GUI gui;

    double _latest_distance, _latest_angle, _initial_x, _initial_y, _initial_yaw;
    bool initial_pose_set = false;

    // Variables to store initial position and orientation
    bool update = false;

    std::mutex _mutex;

public:
    MappingNode();

    // Callback for LiDAR data
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // Callback for Odometry data
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q);
};

#endif // LIDARNODE_H
