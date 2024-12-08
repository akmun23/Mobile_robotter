#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include "lidarnode.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<MappingNode>();

    // Create a multi-threaded executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // Spin the executor to process callbacks in parallel
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
