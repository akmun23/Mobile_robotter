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
    std::lock_guard<std::mutex> lock(_mutex);

    if (!initial_pose_set) {
        return;
    }

    // Prepare for motion compensation
    _latest_angle = msg->angle_min;  // Start angle of the scan

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        double distance = msg->ranges[i];

        if (distance <= msg->range_min || distance >= msg->range_max) {
            _latest_angle += msg->angle_increment;
            continue;
        }

        // Update the GUI with the transformed LiDAR data
        gui.update(update, -_latest_angle - _robot_yaw, distance, -_robot_x, _robot_y, -_robot_yaw);

        _latest_angle += msg->angle_increment;
    }
}



// Callback for Odometry data
void MappingNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(_mutex);

    // Set the initial position only once
    if (!initial_pose_set) {
        _initial_x = msg->pose.pose.position.x;
        _initial_y = msg->pose.pose.position.y;

        // Adjust initial yaw (optional, only for display correction)
        _initial_yaw = getYawFromQuaternion(msg->pose.pose.orientation) - M_PI / 2;

        initial_pose_set = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose set: x=%.2f, y=%.2f, yaw=%.2f",
                    _initial_x, _initial_y, _initial_yaw);
    }

    // Get the current odometry position
    double raw_x = msg->pose.pose.position.x - _initial_x;
    double raw_y = msg->pose.pose.position.y - _initial_y;

    // Apply a rotation to align with the initial yaw
    double cos_initial = std::cos(_initial_yaw);
    double sin_initial = std::sin(_initial_yaw);

    // Transform the coordinates into the initial yaw-aligned frame
    _robot_x = raw_x * cos_initial + raw_y * sin_initial;
    _robot_y = -raw_x * sin_initial + raw_y * cos_initial;

    // Get the robot's current yaw
    double current_yaw = getYawFromQuaternion(msg->pose.pose.orientation);

    _robot_yaw = current_yaw - _initial_yaw;
}

double MappingNode::getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}
