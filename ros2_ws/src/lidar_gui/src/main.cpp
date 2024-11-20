#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include "lidarnode.h"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::cout << "Starting spin" << std::endl;
    rclcpp::spin(std::make_shared<MappingNode>());
    rclcpp::shutdown();
    return 0;
}
