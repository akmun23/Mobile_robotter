# Mobile Robotter
Semesterprojekt i mobile robotter, 3. semester

## Indholdsfortegnelse
- [Setup af ROS på PC](#setup-af-ros-på-pc)
- [Test om ROS virker](#test-om-ros-virker)
- [Klargøring til TurtleBot3](#klargøring-til-turtlebot3)
- [Forbindelse til Raspberry Pi](#forbindelse-til-raspberry-pi)
- [Køring af Controller Input Program](#køring-af-controller-input-program)

## Setup af ROS på PC
**DETTE KAN KUN GØRES PÅ UBUNTU 22.04**
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```
```
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
```
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
```
sudo apt update
```
```
sudo apt upgrade
```
```
sudo apt install ros-humble-desktop
```
```
source /opt/ros/humble/setup.bash
```
# Test om ROS virker
For at kontrollere, om ROS er korrekt installeret, kan du køre følgende kommandoer i separate terminalvinduer:
På en terminal kør:
```
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

På en anden terminal kør:
```
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

Hvis der modtages bekseder på den anden terminal så er ROS sat op

# Klargøring til Turtlebot3
Kør følgende kommandoer i en terminal for at forberede TurtleBot3:
```
sudo apt install ros-humble-gazebo-*
```
```
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
```
```
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```
```
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
```
```
echo 'export ROS_DOMAIN_ID=30 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc
```

Nu er det meste forhåbentligt installeret 

# Forbindelse til Raspberry Pi

For at forbinde din PC til Raspberry Pi og sikre, at de kan kommunikere med ROS 2, følg disse trin:

### 1. Opret forbindelse via SSH
For at oprette forbindelse til Raspberry Pi fra din PC ved hjælp af SSH, skal du følge disse trin:

1. **Find IP-adressen på Raspberry Pi**:
   - Log ind på Raspberry Pi og kør følgende kommando i terminalen for at finde dens IP-adresse:
     ```
     hostname -I
     ```

2. **Opret SSH-forbindelse fra din PC**:
   - Åbn en terminal på din PC og kør følgende kommando for at oprette forbindelse til Raspberry Pi. Erstat `<Raspberry_Pi_IP>` med den faktiske IP-adresse, du fandt tidligere, og `<username>` med brugernavnet på din Raspberry Pi (standard er typisk `pi`):
     ```
     ssh <pi>@<Raspberry_Pi_IP>
     ```
   - Hvis du bliver bedt om det, skal du indtaste din Raspberry Pi's adgangskode.

3. **Indstil ROS_MASTER_URI på PC'en**:
   - På din PC skal du åbne en terminal og sætte `ROS_MASTER_URI` til IP-adressen af Raspberry Pi. Erstat `<Raspberry_Pi_IP>` med den faktiske IP-adresse, du fandt tidligere:
     ```
     export ROS_MASTER_URI=http://<Raspberry_Pi_IP>:11311
     ```
     
4. **Indstil ROS_DOMAIN_ID**:
   - Sørg for, at `ROS_DOMAIN_ID` er det samme på både din PC og Raspberry Pi. For at gøre dette, kør følgende kommando i terminalen på din PC:
     ```
     export ROS_DOMAIN_ID=30
     ```
     Dette skulle gerne være indstillet hvis de forrige trin er fulgt

5. **Kør TurtleBot3 på Raspberry Pi**:
   - Log ind på Raspberry Pi og kør følgende kommando for at starte TurtleBot3:
     ```
     ros2 launch turtlebot3_bringup robot.launch.py
     ```

# Køring af controller input program
Gå til PC terminalen
Gå til den relevante mappe f.eks.:
~/Documents/GitHub/Mobile_robotter/ros2_ws

Vigtigt: Sørg for, at der kun er én mappe, der hedder src. Hvis ikke, skal de andre mapper slettes, da de er bygget til den specifikke computer, der har lavet filerne.

Byg arbejdsområdet:
```
colcon build --symlink-install
```
Denne skal rettes til din mappe sti!
```
source /home/aksel/Documents/GitHub/Mobile_robotter/ros2_ws/install/setup.bash
```
```
ros2 launch controller_input controller_launch.py
```
Hvis der er sat en controller til skulle denne gerne give outputs i terminalen.
Desuden skulle robotten også gerne reagere og køre ved inputs
