#include "dtmf_to_cmd_vel_node.h"
#include "audio.h"  // Include to use Audio class methods

DtmfToCmdVelNode::DtmfToCmdVelNode() : Node("dtmf_to_cmd_vel_node") {
    _audio = std::make_unique<Goertzel>(); // Initialize _audio as a unique pointer
    _publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    Goertzel::setDtmfNode(this);

    _audio->Init();
    _audio->start(); 
}


void DtmfToCmdVelNode::publishTwistMessage(int speed, int direction) {
    auto twistMsg = geometry_msgs::msg::Twist();
    // Normalize speed and direction to be between -1 and 1
    twistMsg.linear.x = (static_cast<float>(speed) / 255) * 2.0 - 1.0;
    twistMsg.linear.x = twistMsg.linear.x/5; 
    twistMsg.angular.z = (static_cast<float>(direction) / 255.0) * 2.0 - 1.0; 
    _publisher->publish(twistMsg);
    printf("\n The robot is driving at speed %f in direction %f \n", twistMsg.linear.x, twistMsg.angular.z );
}

void DtmfToCmdVelNode::publishTwistMessage(bool error){
	auto twistMsg = geometry_msgs::msg::Twist();
	twistMsg.linear.x = 0;
    twistMsg.angular.z = 0;
    _publisher->publish(twistMsg);
    printf("Error received. Robot stopped \n");
    fflush(stdout);   
}


