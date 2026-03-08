#include <gait.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// Humble (C++17) için standart PI tanımı. Artik <numbers> kütüphanesine ihtiyacın yok.
static const double PI = 3.14159265358979323846;

//==============================================================================
// Constructor: Parametreleri deklare et ve al
//==============================================================================

Gait::Gait(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
    // ROS 2 Humble'da parametre deklarasyonu zorunludur.
    node_->declare_parameter("CYCLE_LENGTH", 40);
    node_->declare_parameter("LEG_LIFT_HEIGHT", 0.05);
    node_->declare_parameter("NUMBER_OF_LEGS", 6);
    node_->declare_parameter("GAIT_STYLE", "TRIPOD");

    node_->get_parameter("CYCLE_LENGTH", CYCLE_LENGTH);
    node_->get_parameter("LEG_LIFT_HEIGHT", LEG_LIFT_HEIGHT);
    node_->get_parameter("NUMBER_OF_LEGS", NUMBER_OF_LEGS);
    node_->get_parameter("GAIT_STYLE", GAIT_STYLE);

    cycle_period_ = 0;
    is_travelling_ = false;
    in_cycle_ = false;
    extra_gait_cycle_ = 1;
    
    current_time_ = node_->now();
    last_time_ = node_->now();
    
    gait_factor = 1.0;
    cycle_leg_number_ = {1, 0, 1, 0, 1, 0}; // Varsayılan Tripod

    if (GAIT_STYLE == "RIPPLE")
    {
        gait_factor = 0.5;
        cycle_leg_number_ = {1, 0, 2, 0, 2, 1};
    }
    
    period_distance = 0.0;
    period_height = 0.0;
}

//=============================================================================
// cyclePeriod: Adım yörüngesi ve odometri hesabı
//=============================================================================

void Gait::cyclePeriod(const geometry_msgs::msg::Pose2D & base, 
                       hexapod_msgs::msg::FeetPositions * feet, 
                       geometry_msgs::msg::Twist * gait_vel)
{
    // Küçük harf pi yerine yukarıdaki PI sabitini kullanıyoruz.
    period_height = std::sin(cycle_period_ * PI / CYCLE_LENGTH);

    // Delta time hesaplaması
    current_time_ = node_->now();
    double dt = (current_time_ - last_time_).seconds();
    
    // Güvenlik: dt sıfıra çok yakınsa (döngü çok hızlıysa) odometri sapmasını engelle
    if (dt < 0.0001) dt = 0.02; // 50Hz varsayılan fallback

    gait_vel->linear.x = ((PI * base.x) / CYCLE_LENGTH) * period_height * (1.0 / dt);
    gait_vel->linear.y = ((-PI * base.y) / CYCLE_LENGTH) * period_height * (1.0 / dt);
    gait_vel->angular.z = ((PI * base.theta) / CYCLE_LENGTH) * period_height * (1.0 / dt);
    
    last_time_ = current_time_;

    for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
    {
        // Bacağı kaldır ve ileri taşı (Swing Phase)
        if (cycle_leg_number_[leg_index] == 0 && is_travelling_)
        {
            period_distance = std::cos(cycle_period_ * PI / CYCLE_LENGTH);
            feet->foot[leg_index].position.x = base.x * period_distance;
            feet->foot[leg_index].position.y = base.y * period_distance;
            feet->foot[leg_index].position.z = LEG_LIFT_HEIGHT * period_height;
            feet->foot[leg_index].orientation.yaw = base.theta * period_distance;
        }
        // Bacağı geri iterek gövdeyi ilerlet (Stance Phase)
        else if (cycle_leg_number_[leg_index] == 1)
        {
            period_distance = std::cos(cycle_period_ * PI * gait_factor / CYCLE_LENGTH);
            feet->foot[leg_index].position.x = -base.x * period_distance;
            feet->foot[leg_index].position.y = -base.y * period_distance;
            feet->foot[leg_index].position.z = 0.0;
            feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;
        }
        else if (cycle_leg_number_[leg_index] == 2)
        {
            period_distance = std::cos((CYCLE_LENGTH + cycle_period_) * PI * gait_factor / CYCLE_LENGTH);
            feet->foot[leg_index].position.x = -base.x * period_distance;
            feet->foot[leg_index].position.y = -base.y * period_distance;
            feet->foot[leg_index].position.z = 0.0;
            feet->foot[leg_index].orientation.yaw = -base.theta * period_distance;
        }
    }
}

//=============================================================================
// gaitCycle: Ana yürüyüş sekansiyeri
//=============================================================================

void Gait::gaitCycle(const geometry_msgs::msg::Twist & cmd_vel, 
                     hexapod_msgs::msg::FeetPositions * feet, 
                     geometry_msgs::msg::Twist * gait_vel)
{
    geometry_msgs::msg::Pose2D base; // Pose2_d değil Pose2D
    base.x = cmd_vel.linear.x / PI * CYCLE_LENGTH;
    base.y = cmd_vel.linear.y / PI * CYCLE_LENGTH;
    base.theta = cmd_vel.angular.z / PI * CYCLE_LENGTH;

    // Alçak geçiren filtre (Smooth transition)
    smooth_base_.x = base.x * 0.05 + (smooth_base_.x * 0.95);
    smooth_base_.y = base.y * 0.05 + (smooth_base_.y * 0.95);
    smooth_base_.theta = base.theta * 0.05 + (smooth_base_.theta * 0.95);

    // Hareket kontrolü (Thresholds)
    if ((std::abs(smooth_base_.y) > 0.001) || 
        (std::abs(smooth_base_.x) > 0.001) || 
        (std::abs(smooth_base_.theta) > 0.004)) 
    {
        is_travelling_ = true;
    }
    else
    {
        is_travelling_ = false;
        // Ayakların başlangıç pozisyonuna dönmesini sağla
        for (int i = 0; i < NUMBER_OF_LEGS; i++)
        {
            if (std::abs(feet->foot[i].position.z) > 0.001) {
                extra_gait_cycle_ = CYCLE_LENGTH - cycle_period_ + CYCLE_LENGTH;
                break;
            }
            extra_gait_cycle_ = 1;
        }

        if (extra_gait_cycle_ > 1) {
            extra_gait_cycle_--;
            in_cycle_ = (extra_gait_cycle_ != 1);
        }
    }

    if (is_travelling_ || in_cycle_)
    {
        cyclePeriod(smooth_base_, feet, gait_vel);
        cycle_period_++;
    }
    else
    {
        cycle_period_ = 0;
    }

    if (cycle_period_ >= CYCLE_LENGTH)
    {
        cycle_period_ = 0;
        sequence_change(cycle_leg_number_);
    }
}

void Gait::sequence_change(std::vector<int> & vec)
{
    for (size_t i = 0; i < vec.size(); i++)
    {
        if (vec[i] == 0) vec[i] = 1;
        else if (vec[i] == 1 && GAIT_STYLE == "RIPPLE") vec[i] = 2;
        else vec[i] = 0;
    }
}