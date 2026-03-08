#include <control.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

static const double PI = 3.14159265358979323846; // C++20 bağımlılığından kurtulduk

//==============================================================================
// Constructor
//==============================================================================

Control::Control() : Node("hexapod_controller")
{
    // ROS 2'de deklarasyon hayatidir. Varsayılan değerlerle deklare ediyoruz.
    this->declare_parameter("NUMBER_OF_LEGS", 6);
    this->declare_parameter("NUMBER_OF_LEG_SEGMENTS", 3);
    this->declare_parameter("STANDING_BODY_HEIGHT", 0.1);
    this->declare_parameter("BODY_MAX_ROLL", 0.2);
    
    // Vektör parametreleri için boş varsayılanlar
    this->declare_parameter("COXA_TO_CENTER_X", std::vector<double>{});
    this->declare_parameter("COXA_TO_CENTER_Y", std::vector<double>{});

    this->get_parameter("NUMBER_OF_LEGS", NUMBER_OF_LEGS);
    this->get_parameter("NUMBER_OF_LEG_SEGMENTS", NUMBER_OF_LEG_JOINTS);
    this->get_parameter("STANDING_BODY_HEIGHT", STANDING_BODY_HEIGHT);

    current_time_odometry_ = this->now();
    last_time_odometry_ = this->now();

    // Subscriber Tanımlamaları
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&Control::cmd_velCallback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10, std::bind(&Control::imuCallback, this, std::placeholders::_1));

    state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/state", 10, std::bind(&Control::stateCallback, this, std::placeholders::_1));

    // Publisher Tanımlamaları
    sounds_pub_ = this->create_publisher<hexapod_msgs::msg::Sounds>("/sounds", 10);
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/calculated", 10);
    
    // PascalCase kuralına dikkat: TwistWithCovarianceStamped
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/twist_with_covariance", 10);

    // Header dosyasındaki isimle eşleşmeli: odom_broadcaster
    odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    imu_calibrate_ = this->create_client<std_srvs::srv::Empty>("/imu/calibrate");
}

// ... Mevcut include ve constructor kodlarinin altina ekle ...

//==============================================================================
// Getters / Setters (Hexapod State)
//==============================================================================
void Control::setHexActiveState(bool state) { hex_state_ = state; }
bool Control::getHexActiveState() { return hex_state_; }
void Control::setPrevHexActiveState(bool state) { prev_hex_state_ = state; }
bool Control::getPrevHexActiveState() { return prev_hex_state_; }

//==============================================================================
// partitionCmd_vel: Hizi bolerek yumusak gecis saglar
//==============================================================================
void Control::partitionCmd_vel(geometry_msgs::msg::Twist *cmd_vel)
{
    // VELOCITY_DIVISION parametresini constructor'da aldigindan emin ol
    cmd_vel->linear.x = cmd_vel_incoming_.linear.x / VELOCITY_DIVISION;
    cmd_vel->linear.y = cmd_vel_incoming_.linear.y / VELOCITY_DIVISION;
    cmd_vel->angular.z = cmd_vel_incoming_.angular.z / VELOCITY_DIVISION;
}

//==============================================================================
// publishJointStates: Bacak ve kafa acilarini sensor_msgs'e basar
//==============================================================================
void Control::publishJointStates(const hexapod_msgs::msg::LegsJoints &legs, 
                                const hexapod_msgs::msg::RPY &head, 
                                sensor_msgs::msg::JointState *joint_state)
{
    joint_state->header.stamp = this->now();
    joint_state->name = servo_names_; // Constructor'da doldurulmus olmali
    joint_state->position.clear();

    for (int i = 0; i < NUMBER_OF_LEGS; i++) {
        joint_state->position.push_back(legs.leg[i].coxa);
        joint_state->position.push_back(legs.leg[i].femur);
        joint_state->position.push_back(legs.leg[i].tibia);
    }
    // Eger kafa eklemleri varsa buraya ekle
    joint_state_pub_->publish(*joint_state);
}

//==============================================================================
// publishTwist: Odometri hizini stamped olarak yayinlar
//==============================================================================
void Control::publishTwist(const geometry_msgs::msg::Twist &gait_vel)
{
    geometry_msgs::msg::TwistWithCovarianceStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = "base_link";
    twist_msg.twist.twist = gait_vel;
    // Kovaryans matrisini basitce sifir birakabilirsin veya diyagonal degerler ata
    twist_pub_->publish(twist_msg);
}

//==============================================================================
// Odometry Publisher
//==============================================================================
void Control::publishOdometry(const geometry_msgs::msg::Twist & gait_vel)
{
    current_time_odometry_ = this->now();
    double dt = (current_time_odometry_ - last_time_odometry_).seconds();

    // Güvenlik: dt sıfırsa hesaplama yapma
    if (dt < 0.0001) return;

    double vth = gait_vel.angular.z;
    pose_th_ += vth * dt;

    double vx = gait_vel.linear.x;
    double vy = gait_vel.linear.y;
    
    // Klasik 2D Odometri Hesabı
    pose_x_ += (vx * std::cos(pose_th_) - vy * std::sin(pose_th_)) * dt;
    pose_y_ += (vx * std::sin(pose_th_) + vy * std::cos(pose_th_)) * dt;

    tf2::Quaternion q;
    q.setRPY(0, 0, pose_th_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time_odometry_;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = pose_x_;
    odom.pose.pose.position.y = pose_y_;
    odom.pose.pose.orientation = tf2::toMsg(q); // tf2_geometry_msgs sayesinde

    odom_pub_->publish(odom);
    
    // TF Yayınlama
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header = odom.header;
    odom_tf.child_frame_id = odom.child_frame_id;
    odom_tf.transform.translation.x = pose_x_;
    odom_tf.transform.translation.y = pose_y_;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = odom.pose.pose.orientation;
    
    odom_broadcaster->sendTransform(odom_tf); // Değişken ismi header ile uyumlu!

    last_time_odometry_ = current_time_odometry_;
}

//==============================================================================
// Callbacks ve Diğer Fonksiyonlar
//==============================================================================

void Control::cmd_velCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_vel_incoming_ = *msg;
}

void Control::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // IMU verisi ile gövde dengeleme mantığı buraya gelecek
    // RCLCPP_INFO(this->get_logger(), "IMU verisi aliniyor...");
}

void Control::stateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && !hex_state_) {
        hex_state_ = true;
        auto s_msg = hexapod_msgs::msg::Sounds();
        s_msg.stand = true;
        sounds_pub_->publish(s_msg);
    }
}
