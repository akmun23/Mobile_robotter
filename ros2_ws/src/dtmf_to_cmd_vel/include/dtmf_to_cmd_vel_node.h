#ifndef DTMF_TO_CMD_VEL_NODE_H
#define DTMF_TO_CMD_VEL_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>

// Forward declaration of Audio class
class Goertzel;

class DtmfToCmdVelNode : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _publisher;  // Publisher for cmd_vel
    std::unique_ptr<Goertzel> _audio;  // Audio instance

public:
    DtmfToCmdVelNode(); // Constructor
    void publishTwistMessage(int speed, int direction);
    void publishTwistMessage(bool error);
};

#endif // DTMF_TO_CMD_VEL_NODE_H
