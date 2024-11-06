#include "lidarData.h"

MappingNode() : Node("mapping_node") {
    // QoS for both LiDAR and odometry data
    rclcpp::QoS qos{rclcpp::SensorDataQoS()};

    // Subscription to LiDAR and odometry
    _lidar_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos, std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));

    // Setup of GUI
    QApplication app(argc, argv);

    QSurfaceFormat fmt;
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    Window GUI;

    GUI.show();

    app.exec();

    /*
    _odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos, std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));
    */
}

/*
void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
	_robot_x = msg->pose.pose.position.x;
	_robot_y = msg->pose.pose.position.y;

	// Get orientation in yaw (assuming quaternion, convert to Euler yaw)
	double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
		                    msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
		                        msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
	robot_yaw_ = atan2(siny_cosp, cosy_cosp);  // In radians
}
*/

std::vector<std::vector<double>> scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
	float angle = msg->angle_min;
	std::vector<double> distForGUI, angleForGUI;

	for (size_t i = 1; i < msg->ranges.size(); ++i) {
		float distance = msg->ranges[i];
		
		distForGUI.push_back(distance);
		angleForGUI.push_back(angle);

		angle += msg->angle_increment;
	}

    return [distForGUI, angleForGUI];
}



