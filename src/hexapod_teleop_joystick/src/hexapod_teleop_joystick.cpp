#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <hexapod_teleop_joystick.h> // Header dosyanızda da rclcpp güncellemeleri yapmalısınız

using std::placeholders::_1;

//==============================================================================
// Constructor
//==============================================================================

HexapodTeleopJoystick::HexapodTeleopJoystick() : Node("hexapod_teleop_joystick")
{
    // ROS 2'de parametreleri önce tanımlamalıyız (Default değerlerle)
    this->declare_parameter("STANDUP_BUTTON", 0);
    this->declare_parameter("SITDOWN_BUTTON", 1);
    this->declare_parameter("BODY_ROTATION_BUTTON", 2);
    this->declare_parameter("FORWARD_BACKWARD_AXES", 1);
    this->declare_parameter("LEFT_RIGHT_AXES", 0);
    this->declare_parameter("YAW_ROTATION_AXES", 2);
    this->declare_parameter("PITCH_ROTATION_AXES", 3);
    this->declare_parameter("MAX_METERS_PER_SEC", 0.5);
    this->declare_parameter("MAX_RADIANS_PER_SEC", 1.0);
    this->declare_parameter("NON_TELEOP", false);

    // Parametreleri okuma
    this->get_parameter("STANDUP_BUTTON", STANDUP_BUTTON);
    this->get_parameter("SITDOWN_BUTTON", SITDOWN_BUTTON);
    this->get_parameter("BODY_ROTATION_BUTTON", BODY_ROTATION_BUTTON);
    this->get_parameter("FORWARD_BACKWARD_AXES", FORWARD_BACKWARD_AXES);
    this->get_parameter("LEFT_RIGHT_AXES", LEFT_RIGHT_AXES);
    this->get_parameter("YAW_ROTATION_AXES", YAW_ROTATION_AXES);
    this->get_parameter("PITCH_ROTATION_AXES", PITCH_ROTATION_AXES);
    this->get_parameter("MAX_METERS_PER_SEC", MAX_METERS_PER_SEC);
    this->get_parameter("MAX_RADIANS_PER_SEC", MAX_RADIANS_PER_SEC);
    this->get_parameter("NON_TELEOP", NON_TELEOP);

    state_.data = false;
    imu_override_.data = false;

    // Subscriber ve Publisher Tanımlamaları
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&HexapodTeleopJoystick::joyCallback, this, _1));

    body_scalar_pub_ = this->create_publisher<geometry_msgs::msg::AccelStamped>("/body_scalar", 10);
    head_scalar_pub_ = this->create_publisher<geometry_msgs::msg::AccelStamped>("/head_scalar", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::Bool>("/state", 10);
    imu_override_pub_ = this->create_publisher<std_msgs::msg::Bool>("/imu/imu_override", 10);
}

//==============================================================================
// Joystick Callback
//==============================================================================

void HexapodTeleopJoystick::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    auto current_time = this->get_clock()->now();

    if (joy->buttons[STANDUP_BUTTON] == 1)
    {
        if (state_.data == false) state_.data = true;
    }

    if (joy->buttons[SITDOWN_BUTTON] == 1)
    {
        if (state_.data == true) state_.data = false;
    }

    // Body rotation logic
    if (joy->buttons[BODY_ROTATION_BUTTON] == 1)
    {
        imu_override_.data = true;
        body_scalar_.header.stamp = current_time;
        body_scalar_.accel.angular.x = -joy->axes[LEFT_RIGHT_AXES];
        body_scalar_.accel.angular.y = -joy->axes[FORWARD_BACKWARD_AXES];
        head_scalar_.header.stamp = current_time;
        head_scalar_.accel.angular.z = joy->axes[YAW_ROTATION_AXES];
        head_scalar_.accel.angular.y = joy->axes[PITCH_ROTATION_AXES];
    }
    else
    {
        imu_override_.data = false;
    }

    // Travelling logic
    if (joy->buttons[BODY_ROTATION_BUTTON] != 1)
    {
        cmd_vel_.linear.x = joy->axes[FORWARD_BACKWARD_AXES] * MAX_METERS_PER_SEC;
        cmd_vel_.linear.y = -joy->axes[LEFT_RIGHT_AXES] * MAX_METERS_PER_SEC;
        cmd_vel_.angular.z = joy->axes[YAW_ROTATION_AXES] * MAX_RADIANS_PER_SEC;
    }
}

//==============================================================================
// Main
//==============================================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HexapodTeleopJoystick>();

    rclcpp::WallRate loop_rate(100); // 100 Hz

    while (rclcpp::ok())
    {
        if (node->NON_TELEOP == false)
        {
            node->cmd_vel_pub_->publish(node->cmd_vel_);
            node->body_scalar_pub_->publish(node->body_scalar_);
            node->head_scalar_pub_->publish(node->head_scalar_);
        }
        node->state_pub_->publish(node->state_);
        node->imu_override_pub_->publish(node->imu_override_);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}