# Mobile_robotter
Semesterprojekt i mobile robotter 3. semester

# Setup af ROS på PC
DETTE KAN KUN GØRES PÅ UBUNTU 22.04
Kør disse tung

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt upgrade

sudo apt install ros-humble-desktop

source /opt/ros/humble/setup.bash

# Test om ROS virker
På en terminal kør:
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

På en anden terminal kør:
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener

Hvis der modtages bekseder på den anden terminal så er ROS sat op

# Klargøring til Turtlebot3
Kør disse ting i en terminal:
sudo apt install ros-humble-gazebo-*

sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros

sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

sudo apt remove ros-humble-turtlebot3-msgs
sudo apt remove ros-humble-turtlebot3
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src/
git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/turtlebot3_ws
colcon build --symlink-install
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc

echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc

Nu er det meste forhåbentligt installeret 
