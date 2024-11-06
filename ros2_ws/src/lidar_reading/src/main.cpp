#include "windowClass.h"
#include "lidarData.h"

#include <iostream>
#include <math.h>
#include <QApplication>
#include <QSurfaceFormat>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>

using namespace std;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MappingNode>());
    rclcpp::shutdown();
    return 0;
}
