#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <QtSql/QSqlDatabase>
#include <QtSql/QSqlQuery>
#include <QtSql/QSqlError>
#include <vector>
#include <QDebug>

class MappingNode : public rclcpp::Node {
private:
    // Variables to store current position and a list of scanned points in the global map
    std::vector<double> map_x_points, map_y_points;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _lidar_subscription;
    QSqlDatabase db;

public:
    MappingNode() : Node("mapping_node") {
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
            "/scan", qos, std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));
    }


    // Callback for LiDAR data
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Check if new data can be inserted into the database
        QSqlQuery query;

        if (query.exec("SELECT status FROM lidar_data WHERE id = 1") && query.next()) {
            int status = query.value(0).toInt();
            if (status == 0) {
                // Insert LiDAR data into the database
                float angle = msg->angle_min;
                for (size_t i = 0; i < msg->ranges.size(); ++i) {
                    float distance = msg->ranges[i];

                    // Only insert valid distances
                    if (distance >= msg->range_min && distance <= msg->range_max) {
                        // Prepare SQL statement to insert data
                        query.prepare("INSERT INTO lidar_data (angle, distance) VALUES (:angle, :distance)");
                        query.bindValue(":angle", angle);
                        query.bindValue(":distance", distance);

                        if (!query.exec()) {
                            RCLCPP_ERROR(this->get_logger(), "Error inserting data into lidar_data: %s", query.lastError().text().toStdString().c_str());
                        }
                    }
                    angle += msg->angle_increment;
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

    ~MappingNode() {
        db.close();
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MappingNode>());
    rclcpp::shutdown();
    return 0;
}
