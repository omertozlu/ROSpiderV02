#ifndef SERVO_DRIVER_H_
#define SERVO_DRIVER_H_

#include <cmath>
#include <vector>
#include <string>
#include <memory>

// ROS 2 Temel Başlıkları
#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <sensor_msgs/msg/joint_state.hpp>

//==============================================================================
// ServoDriver Sınıfı: Donanım ve ROS 2 Arasındaki Köprü
//==============================================================================

class ServoDriver
{
    public:
        /**
         * @brief Constructor: Parametre erişimi için Node pointer'ı alır.
         * @param node Ana kontrolcü düğümünün akıllı işaretçisi.
         */
        explicit ServoDriver(std::shared_ptr<rclcpp::Node> node);
        
        ~ServoDriver();

        // ROS 2 Mesaj Tipleri (msg/ namespace zorunludur)
        void transmitServoPositions(const sensor_msgs::msg::JointState &joint_state);
        void makeSureServosAreOn(const sensor_msgs::msg::JointState &joint_state);
        void freeServos();

    private:
        void convertAngles(const sensor_msgs::msg::JointState &joint_state);

        // ROS 2 Düğüm Referansı (Logging ve Parametreler için)
        std::shared_ptr<rclcpp::Node> node_;

        // Dynamixel SDK Nesneleri (Raw pointer yerine akıllı yönetim tercih edilebilir ama SDK yapısı gereği böyle kalabilir)
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;

        uint8_t dxl_error = 0;
        uint16_t currentPos;
        uint8_t param_goal_position[2];

        // Veri Konteynırları
        std::vector<int> cur_pos_;
        std::vector<int> goal_pos_;
        std::vector<int> pose_steps_;
        std::vector<int> write_pos_;
        std::vector<double> OFFSET;
        std::vector<int> ID;
        std::vector<int> TICKS;
        std::vector<int> CENTER;
        std::vector<double> RAD_TO_SERVO_RESOLUTION;
        std::vector<double> MAX_RADIANS;
        std::vector<int> servo_orientation_;
        std::vector<std::string> servo_map_key_;

        // Durum Bayrakları
        bool portOpenSuccess = false;
        bool torque_on = true;
        bool torque_off = true;
        bool writeParamSuccess = true;
        bool servos_free_;
        
        int SERVO_COUNT;
        int TORQUE_ENABLE, PRESENT_POSITION_L, GOAL_POSITION_L, INTERPOLATION_LOOP_RATE;
        
        // Dynamixel Sync Write Parametreleri
        const int LEN_GOAL_POSITION = 2; 
};

#endif // SERVO_DRIVER_H_