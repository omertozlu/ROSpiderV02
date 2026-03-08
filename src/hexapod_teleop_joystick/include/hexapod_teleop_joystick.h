#ifndef HEXAPOD_TELEOP_H_
#define HEXAPOD_TELEOP_H_

// ROS 2 Headerları
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>

// ROS 2'de sınıflar genellikle rclcpp::Node'dan miras alır
class HexapodTeleopJoystick : public rclcpp::Node
{
    public:
        HexapodTeleopJoystick();

        // Mesaj nesneleri (ROS 2 namespace yapısı eklendi)
        std_msgs::msg::Bool state_;
        std_msgs::msg::Bool imu_override_;
        geometry_msgs::msg::AccelStamped body_scalar_;
        geometry_msgs::msg::AccelStamped head_scalar_;
        geometry_msgs::msg::Twist cmd_vel_;

        // Publisher'lar artık SharedPtr tipindedir
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr body_scalar_pub_;
        rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr head_scalar_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr imu_override_pub_;

        bool NON_TELEOP; // cmd_vel yayınını kontrol eder

    private:
        // Callback imzasında SharedPtr kullanımı
        void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);

        // Subscriber SharedPtr tipindedir
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

        // Parametre değişkenleri
        int STANDUP_BUTTON, SITDOWN_BUTTON, BODY_ROTATION_BUTTON;
        int FORWARD_BACKWARD_AXES, LEFT_RIGHT_AXES, YAW_ROTATION_AXES, PITCH_ROTATION_AXES;
        double MAX_METERS_PER_SEC, MAX_RADIANS_PER_SEC;

        // ROS 1'deki NodeHandle (nh_) artık yok, 'this' anahtar kelimesi ile node fonksiyonlarına erişiyoruz.
};

#endif // HEXAPOD_TELEOP_H_