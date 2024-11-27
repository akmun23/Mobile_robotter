#include "rclcpp/rclcpp.hpp"
#include "controllerinput.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<controllerInput>());
    rclcpp::shutdown();
    return 0;
}
