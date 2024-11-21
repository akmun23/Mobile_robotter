#include "lidarnode.h"

MappingNode::MappingNode() : Node("mapping_node")
{
    // QoS for both LiDAR and odometry data
    rclcpp::QoS qos{rclcpp::SensorDataQoS()};

    // Subscription to LiDAR and odometry
    _lidar_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));
    _odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(), std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));

    gui.show();
}

// Callback for LiDAR data
void MappingNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float _latest_angle = msg->angle_min;
    // float noise_threshold = 0.1;

    for (std::size_t i = 0; i < msg->ranges.size(); ++i) {
        float distance = msg->ranges[i];
        if (distance >= msg->range_min && distance <= msg->range_max) {
            _latest_distance = distance;
            gui.update(update, _latest_angle, _latest_distance, _robot_x, _robot_y, _robot_yaw);
        }
        _latest_angle += msg->angle_increment;
    }
}

// Callback for Odometry data
void MappingNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Update robot's current position
    _robot_x = msg->pose.pose.position.x;
    _robot_y = msg->pose.pose.position.y;

    // Extract the yaw from the quaternion and update the robot's yaw
    const auto &q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    _robot_yaw = std::atan2(siny_cosp, cosy_cosp);  // Adjust yaw relative to the initial yaw
}
