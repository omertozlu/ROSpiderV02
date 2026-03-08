#ifndef IK_H_
#define IK_H_

#include <cmath>
#include <vector>
#include <memory>

// ROS 2 Temel Başlıkları
#include <rclcpp/rclcpp.hpp>

// ROS 2 Mesaj Tipleri (.hpp uzantısı ve msg/ namespace kullanımı zorunludur)
#include <hexapod_msgs/msg/pose.hpp>
#include <hexapod_msgs/msg/legs_joints.hpp>
#include <hexapod_msgs/msg/feet_positions.hpp>

//=============================================================================
// Trigonometri Veri Yapısı: Sinüs ve kosinüsü bir arada tutar
//=============================================================================
struct Trig
{
    double sine;
    double cosine;
};

//=============================================================================
// Ik Sınıfı: Ters Kinematik Çözücü
//=============================================================================
class Ik
{
    public:
        /**
         * @brief Constructor: Parametreleri okumak için bir Node pointer'ı gerektirir.
         * @param node ROS 2 düğümüne akıllı işaretçi.
         */
        explicit Ik(std::shared_ptr<rclcpp::Node> node);

        /**
         * @brief IK Çözümü: Ayak pozisyonlarından eklem açılarını hesaplar.
         * @param feet Ayakların dünya/gövde koordinatındaki pozisyonları.
         * @param body Gövdenin oryantasyonu ve yüksekliği.
         * @param legs Hesaplanan eklem açılarının yazılacağı çıktı mesajı.
         */
        void calculateIK(const hexapod_msgs::msg::FeetPositions &feet, 
                         const hexapod_msgs::msg::Pose &body, 
                         hexapod_msgs::msg::LegsJoints::SharedPtr legs);

    private:
        Trig getSinCos(double angle_rad);

        // Parametre yönetimi için düğüm referansı
        std::shared_ptr<rclcpp::Node> node_;

        // Robotun fiziksel ölçüleri (Parametrelerden okunur)
        std::vector<double> COXA_TO_CENTER_X, COXA_TO_CENTER_Y;
        std::vector<double> INIT_COXA_ANGLE;
        std::vector<double> INIT_FOOT_POS_X, INIT_FOOT_POS_Y, INIT_FOOT_POS_Z;
        
        double COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, TARSUS_LENGTH;
        int NUMBER_OF_LEGS;
};

#endif // IK_H_