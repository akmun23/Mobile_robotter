#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <iostream>

class ControllerInput : public rclcpp::Node
{
public:
    ControllerInput()
        : Node("controller_input")
    {
        // Create a publisher for cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscribe to the joystick messages
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ControllerInput::joy_callback, this, std::placeholders::_1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Create a Twist message for the robot
        geometry_msgs::msg::Twist twist_msg;

        // Assign joystick axes to the Twist message
        twist_msg.linear.x = msg->axes[1];  // Assuming axis 1 is forward/backward
        twist_msg.angular.z = msg->axes[0];  // Assuming axis 0 is left/right

        // Publish the Twist message
        publisher_->publish(twist_msg);

        // Add more detailed output based on joystick button presses if needed
        if (msg->buttons[0] == 1) {
            std::cout << "Button X Pressed" << std::endl; // Replace with your button index
        }
        else if (msg->buttons[1] == 1) {
            std::cout << "Button O Pressed" << std::endl; // Replace with your button index
        }
        else if (msg->buttons[2] == 1) {
            std::cout << "Button ∆  Pressed" << std::endl; // Replace with your button index
        }
        else if (msg->buttons[3] == 1) {
            std::cout << "Button □ Pressed" << std::endl; // Replace with your button index
        } else {
                // Output joystick values and the corresponding cmd_vel values
        std::cout << "Joystick Axes: "
                  << "Linear: " << twist_msg.linear.x 
                  << ", Angular: " << twist_msg.angular.z << std::endl;
        }                
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerInput>());
    rclcpp::shutdown();
    return 0;
}
