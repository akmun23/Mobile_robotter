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

MappingNode::~MappingNode() {}

// Callback for LiDAR data
void MappingNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float distance = msg->ranges[i];
        if (distance >= msg->range_min && distance <= msg->range_max) {
            // Calculate the point in the "base_link" frame
            _distance.push_back(distance);
            _angle.push_back(angle);

            // Send the transformed points to the GUI
            gui.update(update, angle, distance*100);
        }
        angle += msg->angle_increment;
    }
}

// Callback for Odometry data
void MappingNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Set the initial position only once
    if (!initial_pose_set) {
        _initial_x = msg->pose.pose.position.x;
        _initial_y = msg->pose.pose.position.y;

        // Extract yaw from quaternion
        const auto &q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        _initial_yaw = std::atan2(siny_cosp, cosy_cosp);

        initial_pose_set = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose set: x=%.2f, y=%.2f, yaw=%.2f",
                    _initial_x, _initial_y, _initial_yaw);
    }

    // Update robot's current position
    _robot_x = msg->pose.pose.position.x - _initial_x;
    _robot_y = msg->pose.pose.position.y - _initial_y;

    // Extract the yaw from the quaternion and update the robot's yaw
    const auto &q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    _robot_yaw = std::atan2(siny_cosp, cosy_cosp) - _initial_yaw;  // Adjust yaw relative to the initial yaw
}
