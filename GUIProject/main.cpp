#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include "lidarnode.h"
#include "GUIWindow.h"

int main() {
    rclcpp::init(argc, argv);
    std::cout << "Starting spin" << std::endl;
    rclcpp::spin(std::make_shared<MappingNode>());
    rclcpp::shutdown();

    return 0;
}
