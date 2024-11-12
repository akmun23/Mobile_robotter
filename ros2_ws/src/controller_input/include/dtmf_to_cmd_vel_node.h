#ifndef DTMF_TO_CMD_VEL_NODE_H
#define DTMF_TO_CMD_VEL_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include "audio.h"  // Ensure this is included to get the Audio definition

class DtmfToCmdVelNode : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;  // Publisher for cmd_vel
    std::unique_ptr<Audio> _audio;  // Use unique pointer for Audio

public:
    DtmfToCmdVelNode();  // Constructor declaration
    ~DtmfToCmdVelNode(); // Destructor declaration
    void publishTwistMessage(int speed, int direction);
};

#endif // DTMF_TO_CMD_VEL_NODE_H

