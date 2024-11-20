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

    // QoS for both LiDAR and odometry data
    rclcpp::QoS qos{rclcpp::SensorDataQoS()};

    // Subscription to LiDAR and odometry
    _lidar_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));
    _odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(), std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));
}

MappingNode::~MappingNode() {
    db.close();
}

// Callback for LiDAR data
void MappingNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float angle = msg->angle_min;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        float distance = msg->ranges[i];

        if (distance >= msg->range_min && distance <= msg->range_max) {
            // Calculate the point in the "base_link" frame
            _distance.push_back(distance);
            _angle.push_back(angle);
        }
        angle += msg->angle_increment;
    }

    // Send the transformed points to the GUI
    sendMapDataToGUI(_distance, _angle, _robot_x, _robot_y, _robot_yaw);
}

// Callback for Odometry data
void MappingNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Set the initial position only once
    if (!initial_pose_set) {
        _initial_x = msg->pose.pose.position.x;
        _initial_y = msg->pose.pose.position.y;

        // Extract yaw from quaternion
        const auto &q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        _initial_yaw = std::atan2(siny_cosp, cosy_cosp);

        initial_pose_set = true;
        RCLCPP_INFO(this->get_logger(), "Initial pose set: x=%.2f, y=%.2f, yaw=%.2f",
                    _initial_x, _initial_y, _initial_yaw);
    }

    // Update robot's current position
    _robot_x = msg->pose.pose.position.x - _initial_x;
    _robot_y = msg->pose.pose.position.y - _initial_y;

    // Extract the yaw from the quaternion and update the robot's yaw
    const auto &q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    _robot_yaw = std::atan2(siny_cosp, cosy_cosp) - _initial_yaw;  // Adjust yaw relative to the initial yaw
}

void MappingNode::sendMapDataToGUI(std::vector<double>& distance, std::vector<double>& angle,
                                   double robot_x, double robot_y, double robot_yaw) {
    QSqlQuery query;

    if (query.exec("SELECT status FROM lidar_data WHERE id = 1") && query.next()) {
        int status = query.value(0).toInt();
        if (status == 0) {
            // Insert LiDAR data into the database
            for (size_t i = 0; i < distance.size(); ++i) {
                // Prepare SQL statement to insert data
                query.prepare("INSERT INTO lidar_data (angle, distance) VALUES (:angle, :distance)");
                query.bindValue(":angle", angle[i]);
                query.bindValue(":distance", distance[i]);

                if (!query.exec()) {
                    RCLCPP_ERROR(this->get_logger(), "Error inserting data into lidar_data: %s", query.lastError().text().toStdString().c_str());
                }
            }

            // After inserting the data, update the status to 1 (new data)
            query.prepare("UPDATE lidar_data SET `status` = 1 WHERE id = 1");
            if (!query.exec()) {
                RCLCPP_ERROR(this->get_logger(), "Error updating status in lidar_data: %s", query.lastError().text().toStdString().c_str());
            }
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error checking status from lidar_data: %s", query.lastError().text().toStdString().c_str());
    }
}
