#include "dtmf_to_cmd_vel_node.h" 
#include "audio.h"

#include <fstream>   // Only needed if you want to read from a file

#include <iostream>
#include <vector>
#include <complex>
#include <cmath>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char **argv) {  
    rclcpp::init(argc, argv); // Initialize ROS 2
    auto node = std::make_shared<DtmfToCmdVelNode>(); // Create the node
    rclcpp::spin(node); // Start spinning to process callbacks
    rclcpp::shutdown(); // Clean up and shutdown ROS 2
}
