#ifndef GAIT_H_
#define GAIT_H_

#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <memory>

// ROS 2 Temel Başlıkları
#include <rclcpp/rclcpp.hpp>

// ROS 2 Mesaj Tipleri (DOĞRU YAZIM: küçük harf ve alt çizgi)
#include <hexapod_msgs/msg/feet_positions.hpp>
#include <geometry_msgs/msg/pose2_d.hpp> // Pose2D.hpp -> pose2_d.hpp
#include <geometry_msgs/msg/twist.hpp>

//=============================================================================
// Gait Sınıfı: Yürüyüş Sekansiyeri
//=============================================================================
class Gait
{
    public:
        /**
         * @brief Constructor: Düğüm referansı alır.
         * @param node Parametre erişimi için şart.
         */
        explicit Gait(std::shared_ptr<rclcpp::Node> node);

        void gaitCycle(const geometry_msgs::msg::Twist &cmd_vel, 
                        hexapod_msgs::msg::FeetPositions *feet, 
                        geometry_msgs::msg::Twist *gait_vel);

    private:
        // Tip ismi Pose2D'dir ama include dosyası pose2_d.hpp'dir
        void cyclePeriod(const geometry_msgs::msg::Pose2D &base, 
                          hexapod_msgs::msg::FeetPositions *feet, 
                          geometry_msgs::msg::Twist *gait_vel);
        
        void sequence_change(std::vector<int> &vec);

        std::shared_ptr<rclcpp::Node> node_;

        geometry_msgs::msg::Pose2D smooth_base_; // Pose2_d değil Pose2D
        rclcpp::Time current_time_, last_time_;

        bool is_travelling_;
        bool in_cycle_;
        
        int CYCLE_LENGTH;
        int NUMBER_OF_LEGS;
        double LEG_LIFT_HEIGHT;
        std::string GAIT_STYLE;

        int cycle_period_;
        int extra_gait_cycle_;
        double period_distance;
        double period_height;
        double gait_factor;
        
        std::vector<int> cycle_leg_number_; 
};

#endif // GAIT_H_