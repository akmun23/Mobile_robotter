#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

class MappingNode : public rclcpp::Node {
public:
    MappingNode() : Node("mapping_node") {
        // QoS for both LiDAR and odometry data
        rclcpp::QoS qos{rclcpp::SensorDataQoS()};

        // Subscription to LiDAR and odometry
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos, std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));

        // Initial plot setup
        plt::figure_size(800, 800);
        plt::title("Robot Mapping Visualization");
    }

private:
    // Variables to store current position and a list of scanned points in the global map
    double robot_x_ = 0.0, robot_y_ = 0.0, robot_yaw_ = 0.0;
    std::vector<double> map_x_points, map_y_points;

    // Callback for LiDAR data
	void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
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
		        double global_x = robot_x_ + local_x * cos(robot_yaw_) - local_y * sin(robot_yaw_);
		        double global_y = robot_y_ + local_x * sin(robot_yaw_) + local_y * cos(robot_yaw_);

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
	plt::plot({robot_x_}, {robot_y_}, "ro");
	plt::xlim(-10, 10);  // Set to a fixed global map view
	plt::ylim(-10, 10);
	plt::pause(0.01);
	}

    // Callback for Odometry data
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        // Get orientation in yaw (assuming quaternion, convert to Euler yaw)
        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                                msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                                    msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
        robot_yaw_ = atan2(siny_cosp, cosy_cosp);  // In radians
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MappingNode>());
    rclcpp::shutdown();
    return 0;
}

