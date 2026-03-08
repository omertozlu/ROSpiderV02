#ifndef CONTROL_H_
#define CONTROL_H_

#include <cmath>
#include <memory>
#include <vector>
#include <string>

// ROS 2 Temel Kütüphaneleri
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

// Standart ROS 2 Mesajları (Dosya isimleri her zaman küçük harf ve alt çizgi!)
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Özel Mesajlar
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/msg/rpy.hpp>
#include <hexapod_msgs/msg/legs_joints.hpp>
#include <hexapod_msgs/msg/feet_positions.hpp>
#include <hexapod_msgs/msg/sounds.hpp>

class Control : public rclcpp::Node
{
    public:
        Control();
        void setHexActiveState(bool state);
        bool getHexActiveState();
        void setPrevHexActiveState(bool state);
        bool getPrevHexActiveState();
        
        void publishJointStates(const hexapod_msgs::msg::LegsJoints &legs, 
                                const hexapod_msgs::msg::RPY &head, 
                                sensor_msgs::msg::JointState *joint_state);
        void publishOdometry(const geometry_msgs::msg::Twist &gait_vel);
        void publishTwist(const geometry_msgs::msg::Twist &gait_vel);
        void partitionCmd_vel(geometry_msgs::msg::Twist *cmd_vel);

        int MASTER_LOOP_RATE;
        sensor_msgs::msg::JointState joint_state_;
        hexapod_msgs::msg::Pose body_;
        hexapod_msgs::msg::RPY head_;
        hexapod_msgs::msg::LegsJoints legs_;
        hexapod_msgs::msg::FeetPositions feet_;
        double STANDING_BODY_HEIGHT;
        geometry_msgs::msg::Twist gait_vel_;
        geometry_msgs::msg::Twist cmd_vel_;

    private:
        void cmd_velCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void bodyCallback(const geometry_msgs::msg::AccelStamped::SharedPtr msg);
        void headCallback(const geometry_msgs::msg::AccelStamped::SharedPtr msg);
        void stateCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void imuOverrideCallback(const std_msgs::msg::Bool::SharedPtr msg);
        void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

        hexapod_msgs::msg::Sounds sounds_;
        std_msgs::msg::Bool imu_override_;
        bool imu_init_stored_;
        double imu_roll_lowpass_, imu_pitch_lowpass_, imu_yaw_lowpass_, imu_roll_init_, imu_pitch_init_;
        double MAX_BODY_ROLL_COMP, MAX_BODY_PITCH_COMP, COMPENSATE_INCREMENT, COMPENSATE_TO_WITHIN;
        double BODY_MAX_ROLL, BODY_MAX_PITCH, BODY_MAX_YAW, HEAD_MAX_YAW, HEAD_MAX_PITCH;
        double VELOCITY_DIVISION;
        double pose_x_ = 0.0;
        double pose_y_ = 0.0;
        double pose_th_ = 0.0;
        int NUMBER_OF_LEGS;
        int NUMBER_OF_HEAD_JOINTS;
        int NUMBER_OF_LEG_JOINTS;

        std::vector<std::string> servo_map_key_;
        std::vector<std::string> servo_names_;
        std::vector<int> servo_orientation_;
        bool hex_state_;
        bool prev_hex_state_;

        rclcpp::Time current_time_odometry_, last_time_odometry_, current_time_cmd_vel_, last_time_cmd_vel_;
        
        // Önceki hatanı düzeltmek için değişken ismini odom_broadcaster olarak tutuyorum, .cpp'de de bunu kullan
        std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
        geometry_msgs::msg::Twist cmd_vel_incoming_;

        // Subscriberlar
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr body_scalar_sub_;
        rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr head_scalar_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr imu_override_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

        // Publisherlar
        rclcpp::Publisher<hexapod_msgs::msg::Sounds>::SharedPtr sounds_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        
        // DOĞRU YAZIM: TwistWithCovarianceStamped (PascalCase)
        rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;

        // Servisler
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr imu_calibrate_;
};

#endif // CONTROL_H_