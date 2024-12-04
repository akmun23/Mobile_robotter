#include "lidarnode.h"

MappingNode::MappingNode() : Node("mapping_node")
{
  // Initialize the database connection
  db = QSqlDatabase::addDatabase("QMYSQL");
  db.setHostName("localhost");  // Set the hostname
  db.setDatabaseName("lidar_db");  // Set your database name
  db.setUserName("aksel");  // Set MySQL username
  db.setPassword("hua28rdc");  // Set MySQL password

  if (!db.open()) {
      RCLCPP_ERROR(this->get_logger(), "Database error: %s", db.lastError().text().toStdString().c_str());
  }

  //Delete all data from the databse
  QSqlQuery query;
  if (!query.exec("DELETE FROM lidar_data")) {
    qDebug() << "Error deleting data from lidar_data" << query.lastError().text();
  }


  // QoS for both LiDAR and odometry data
  rclcpp::QoS qos{rclcpp::SensorDataQoS()};

  // Subscription to LiDAR and odometry
  _lidar_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));
  _odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(), std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));

  gui.show();
}

MappingNode::~MappingNode() {
  db.close();
}

// Callback for LiDAR data
void MappingNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  double _latest_angle = msg->angle_min;
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double distance = msg->ranges[i];
      if (distance >= msg->range_min && distance <= msg->range_max) {
          _latest_distance = distance;
          gui.update(update, _latest_angle, _latest_distance, _robot_x, _robot_y, _robot_yaw);
          sendMapDataToGUI(_latest_distance, _latest_angle, _robot_x, _robot_y, _robot_yaw);
      }
      _latest_angle += msg->angle_increment;
  }
}

// Callback for Odometry data
void MappingNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Set the initial position only once
    if (!initial_pose_set) {
        _initial_x = msg->pose.pose.position.x;
        _initial_y = msg->pose.pose.position.y;

        initial_pose_set = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose set: x=%.2f, y=%.2f",
                    _initial_x, _initial_y);
    }

    // Update robot's current position
    _robot_x = msg->pose.pose.position.x - _initial_x;
    _robot_y = -msg->pose.pose.position.y - _initial_y;

    // Extract the yaw from the quaternion and update the robot's yaw
    const auto &q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    _robot_yaw = std::atan2(siny_cosp, cosy_cosp);  // Adjust yaw relative to the initial yaw
}

void MappingNode::sendMapDataToGUI(double& distance, double& angle,
                                   double& robot_x, double& robot_y, double& robot_yaw) {
  QSqlQuery query;
  query.prepare("INSERT INTO lidar_data (distance, angle, robot_x, robot_y, robot_yaw) "
                "VALUES (:distance, :angle, :robot_x, :robot_y, :robot_yaw)");
  query.bindValue(":distance", distance);
  query.bindValue(":angle", angle);
  query.bindValue(":robot_x", robot_x);
  query.bindValue(":robot_y", robot_y);
  query.bindValue(":robot_yaw", robot_yaw);

  if (!query.exec()) {
    qDebug() << "Error inserting data into lidar_data" << query.lastError().text();
  }
}
