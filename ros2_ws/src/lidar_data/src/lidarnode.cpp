#include "lidarnode.h"

MappingNode::MappingNode() : Node("mapping_node") {
    // QoS for both LiDAR and odometry data
    rclcpp::QoS qos{rclcpp::SensorDataQoS()};

    // Subscription to LiDAR and odometry
    _lidar_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos, std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));
    _odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos, std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));

    // Initial plot setup
    plt::figure_size(800, 800);
    plt::title("Robot Mapping Visualization");
}

void MappingNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<double> x_points, y_points;
    float noise_threshold = 0.2;  // Distance difference threshold for noise filtering
    float angle = msg->angle_min;

    // Clear previous scan points
    x_points.clear();
    y_points.clear();

    float previous_distance = msg->ranges[0];
    for (size_t i = 1; i < msg->ranges.size(); ++i) {
        float distance = msg->ranges[i];
        if (distance >= msg->range_min && distance <= msg->range_max) {
            // Noise filtering
            if (abs(distance - previous_distance) < noise_threshold) {
                // Transform LiDAR points to global coordinates
                double local_x = distance * cos(angle);
                double local_y = distance * sin(angle);
                double global_x = _robot_x + local_x * cos(_robot_yaw) - local_y * sin(_robot_yaw);
                double global_y = _robot_y + local_x * sin(_robot_yaw) + local_y * cos(_robot_yaw);

                // Store in the persistent map
                map_x_points.push_back(global_x);
                map_y_points.push_back(global_y);

                const size_t max_points = 10000;  // Maximum number of points to keep

                if (map_x_points.size() > max_points) {
                    map_x_points.erase(map_x_points.begin(), map_x_points.begin() + (map_x_points.size() - max_points));
                    map_y_points.erase(map_y_points.begin(), map_y_points.begin() + (map_y_points.size() - max_points));
                }

                x_points.push_back(global_x);
                y_points.push_back(global_y);
            }
        }
        previous_distance = distance;
        angle += msg->angle_increment;
    }

    // Plot persistent map and current LiDAR scan
    plt::clf();
    plt::scatter(map_x_points, map_y_points, 1, {{"color", "gray"}});
    plt::scatter(x_points, y_points, 1, {{"color", "blue"}});

    // Plot robot's current position in red
    plt::plot({_robot_x}, {_robot_y}, "ro");
    plt::xlim(-10, 10);  // Set to a fixed global map view
    plt::ylim(-10, 10);
    plt::pause(0.01);
}

void MappingNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    _robot_x = msg->pose.pose.position.x;
    _robot_y = msg->pose.pose.position.y;

    // Get orientation in yaw (assuming quaternion, convert to Euler yaw)
    double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                            msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    _robot_yaw = atan2(siny_cosp, cosy_cosp);  // In radians
}
