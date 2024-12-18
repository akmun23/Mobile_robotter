#include "controllerinput.h"

controllerInput::controllerInput() : Node("controller_input")
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

    if (!query.exec("DELETE FROM robot_positions")) {
        qDebug() << "Error deleting data from robot_positions" << query.lastError().text();
    }

    // Subscribe to the joystick messages
    _joySubscriber = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&controllerInput::joy_callback, this, std::placeholders::_1));

    // QoS for both LiDAR and odometry data
    rclcpp::QoS qos{rclcpp::SensorDataQoS()};

    // Subscription to LiDAR and odometry
    _lidar_subscription = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), std::bind(&controllerInput::scanCallback, this, std::placeholders::_1));
    _odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SensorDataQoS(), std::bind(&controllerInput::odomCallback, this, std::placeholders::_1));

    makeAmplitudeFading();
}

controllerInput::~controllerInput() {
    db.close();
}

void controllerInput::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    _latest_scan = msg;
}

void controllerInput::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    if(!_initial_pose_set){
        _initial_x = msg->pose.pose.position.x;
        _initial_y = msg->pose.pose.position.y;
        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        _initial_yaw = std::atan2(siny_cosp, cosy_cosp);

        _cos_initial = std::cos(_initial_yaw);
        _sin_initial = std::sin(_initial_yaw);

        _initial_pose_set = true;

        std::cout << _initial_x << std::endl;
        std::cout << _initial_y << std::endl;
        std::cout << _initial_yaw << std::endl;
    }

    _latest_odom = msg;
}

void controllerInput::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    QSqlQuery query;

    // Map joystick values to the range [0, 100]
    uint8_t linear_value = mapAxisToByte(msg->axes[1]);  // Linear velocity value
    uint8_t angular_value = mapAxisToByte(msg->axes[0]); // Angular velocity value

    // Update the active status based on whether any joystick axes are non-zero
    _joystickActive = (msg->axes[0] != 0.0f || msg->axes[1] != 0.0f);

    // Save the values to member variables for use in the timer callback
    _latestLinearValue = linear_value;
    _latestAngularValue = angular_value;

    if(msg->buttons[0] == 1){
        _currentPress = true;
    }

    if(msg->buttons[0] == 0 && _currentPress){
        play_dtmf_if_active();
        _currentPress = false;
    }

    // Log odom and scan into the database if button 2 is pressed
    if (msg->buttons[1] == 1) {
        std::cout << "Updating database" << std::endl;
        if (_latest_scan && _latest_odom) {
            const auto &q = _latest_odom->pose.pose.orientation;
            double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
            double robot_yaw = std::atan2(siny_cosp, cosy_cosp);

            double raw_x = _latest_odom->pose.pose.position.x - _initial_x;
            double raw_y = _latest_odom->pose.pose.position.y - _initial_y;

            double robot_x = raw_x * _cos_initial + raw_y * _sin_initial;
            double robot_y = -raw_x * _sin_initial + raw_y * _cos_initial;
            robot_yaw = robot_yaw - _initial_yaw;

            // Prepare the insert statement
            QString queryStr = "INSERT INTO lidar_data (distance, angle, robot_x, robot_y, robot_yaw) VALUES ";
            QStringList valueStrings;

            double angle = _latest_scan->angle_min;
            for (size_t i = 0; i < _latest_scan->ranges.size(); ++i) {
                double distance = _latest_scan->ranges[i];
                if (distance >= _latest_scan->range_min && distance <= _latest_scan->range_max) {
                    valueStrings.append(
                        QString("(%1, %2, %3, %4, %5)")
                            .arg(distance)
                            .arg(-angle - robot_yaw)
                            .arg(-robot_x)
                            .arg(robot_y)
                            .arg(-robot_yaw)
                        );
                }
                angle += _latest_scan->angle_increment;
            }

            // Execute the batch insert
            queryStr += valueStrings.join(", ");
            if (!query.exec(queryStr)) {
                qDebug() << "Error inserting data into lidar_data:" << query.lastError().text();
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "No data to log");
        }
    }
}

void controllerInput::play_dtmf_if_active() {
    // Debug print to show the mapped values
    std::cout << "Playing DTMF for Linear Value: " << static_cast<int>(_latestLinearValue)
              << ", Angular Value: " << static_cast<int>(_latestAngularValue) << std::endl;

    // This is needed dont touch
    sf::SoundBuffer buffer;
    std::vector<sf::Int16> samples;
    for (int i = 0; i < 2; i++) {
        samples.push_back(SineWave(i, 200, 0));
    }
    buffer.loadFromSamples(&samples[0], samples.size(), 1, AudioPlayRate);
    sf::Sound sound;
    sound.setBuffer(buffer);
    sound.play();
    usleep(1);
    samples.clear();
    // Ends here

    // Play the dtmf sequence from the joystick input
    playDTMFSequence(_latestLinearValue, _latestAngularValue);
}

void controllerInput::makeAmplitudeFading(){
    double AudioStart = 0;
    double fadeInEnd = soundFrequency * ToneDuration / 1000 / 6;  // 1/6th of tone duration
    double fadeOutBegin = soundFrequency * ToneDuration / 1000 - fadeInEnd;  // 5/6th of tone duration
    double fadeOutEnd = soundFrequency * ToneDuration / 1000;
    double End = soundFrequency * (ToneDuration + PauseDuration) / 1000;

    for (int i = AudioStart; i < fadeInEnd; i++) {
        AmplitudeFading.push_back(i/fadeInEnd);
    }
    for (int i = fadeInEnd; i < fadeOutBegin; i++) {
        AmplitudeFading.push_back(1.0);
    }
    for (int i = fadeOutBegin; i < fadeOutEnd; i++) {
        AmplitudeFading.push_back((fadeOutEnd-i)/(fadeOutEnd-fadeOutBegin));
    }
    for (int i = fadeOutEnd; i < End; i++) {
        AmplitudeFading.push_back(0);
    }

    std::cout << "AmplitudeFading size: " << AmplitudeFading.size() << std::endl;
}

void controllerInput::playTone(double freq1, double freq2, bool lastTone = false){
    sf::SoundBuffer buffer;
    std::vector<sf::Int16> samples;

    double amp = 0.5;

    int time = SamplesPerFrame;
    int sleep = (1000/(AudioPlayRate/SamplesPerFrame))*1000;    // AudioPlayRate/SamplesPerFrame = 7.35 which is times it is sent per sec
        // This 1000 divided by this gives the time in ms it is sent then times 1000 to get mikrosec

    for (int i = 0; i < time; i++) {
        samples.push_back(SineWave(i, freq1, amp * AmplitudeFading[i]) + SineWave(i, freq2, amp*AmplitudeFading[i]));
    }

    buffer.loadFromSamples(&samples[0], samples.size(), 1, AudioPlayRate);

    sf::Sound sound;
    sound.setBuffer(buffer);
    sound.play();
    if(!lastTone){
        usleep(sleep);
    } else {
        usleep(sleep+25000);
    }
    samples.clear();
}

uint8_t controllerInput::mapAxisToByte(float axis_value) {
    return static_cast<uint8_t>(round((axis_value + 1.0) / 2.0 * 100));
}

// Function to play the DTMF sequence for the joystick input
void controllerInput::playDTMFSequence(uint8_t linear_value, uint8_t angular_value) {
    // Play the start tone (e.g., DTMF for '*')
    playTone(941, 1209);

    // Play two tones for the linear velocity
    int linear_tone1 = (linear_value >> 4) & 0x0F;  // Higher nibble (4 bits)
    std::cout << linear_tone1 << std::endl;
    int linear_tone2 = linear_value & 0x0F;          // Lower nibble (4 bits)
    std::cout << linear_tone2 << std::endl;
    playTone(LOW_FREQ[linear_tone1], HIGH_FREQ[linear_tone1]);
    playTone(LOW_FREQ[linear_tone2], HIGH_FREQ[linear_tone2]);

    // Play two tones for the angular velocity
    int angular_tone1 = (angular_value >> 4) & 0x0F; // Higher nibble (4 bits)
    int angular_tone2 = angular_value & 0x0F;        // Lower nibble (4 bits)
    playTone(LOW_FREQ[angular_tone1], HIGH_FREQ[angular_tone1]);
    std::cout << angular_tone1 << std::endl;
    playTone(LOW_FREQ[angular_tone2], HIGH_FREQ[angular_tone2]);
    std::cout << angular_tone2 << std::endl;

    // Play the stop tone (e.g., DTMF for '#')
    playTone(941, 1477, true);
}

double controllerInput::SineWave(int time, double freq, double amp) {
    double result = 0;
    double tpc = AudioPlayRate / freq; // ticks per cycle
    double cycles = time / tpc;
    double rad = 2 * M_PI * cycles;
    double amplitude = 32767 * amp;
    result = amplitude * sin(rad);
    return result;
}
