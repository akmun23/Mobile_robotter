#include "audio.h"

//#include <ros/ros.h>
//#include <geometry_msgs/Twist.h>

int main() {
    /*
    // Initialize ROS node
    ros::init(argc, argv, "dtmf_to_cmd_vel");
    ros::NodeHandle nh;

    // Set up publisher
    cmdVelPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    */

	// Setup of audio
    Audio audio;
    audio.Init();
    audio.start();
    audio.end();



    return EXIT_SUCCESS;

}


