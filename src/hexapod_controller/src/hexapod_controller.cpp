#include <rclcpp/rclcpp.hpp>
#include <control.h>
#include <gait.h>
#include <ik.h>
#include <servo_driver.h>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 1. Ana Düğümü (Node) SharedPtr olarak oluştur
    auto control_node = std::make_shared<Control>();

    // 2. Diğer sınıfları oluştururken düğüm referansını GEÇ
    // Bu adım olmazsa Gait, Ik ve ServoDriver parametre okuyamaz!
    Gait gait(control_node);
    Ik ik(control_node);
    ServoDriver servoDriver(control_node);

    // IK sonuçlarını tutmak için SharedPtr mesajı (Pointer hatasını çözer)
    auto legs_res = std::make_shared<hexapod_msgs::msg::LegsJoints>();

    // Başlangıç yörüngesini hesapla
    gait.gaitCycle(control_node->cmd_vel_, &control_node->feet_, &control_node->gait_vel_);
    ik.calculateIK(control_node->feet_, control_node->body_, legs_res);
    
    // Veriyi control_node içindeki nesneye geri kopyala (Uyum için)
    control_node->legs_ = *legs_res;
    
    control_node->publishJointStates(control_node->legs_, control_node->head_, &control_node->joint_state_);
    control_node->publishOdometry(control_node->gait_vel_);

    // 3. Executor ve Döngü Hızı
    rclcpp::Rate loop_rate(control_node->MASTER_LOOP_RATE);

    RCLCPP_INFO(control_node->get_logger(), "Hexapod Kontrolcü Çalışıyor...");

    while (rclcpp::ok())
    {
        // 4. Callback'leri işle (Gelen joystick verisi vb.)
        rclcpp::spin_some(control_node);

        control_node->partitionCmd_vel(&control_node->cmd_vel_);

        // DURUM: AYAĞA KALKMA
        if (control_node->getHexActiveState() && !control_node->getPrevHexActiveState())
        {
            RCLCPP_INFO(control_node->get_logger(), "Hexapod ayaga kalkiyor...");
            while (control_node->body_.position.z < control_node->STANDING_BODY_HEIGHT)
            {
                control_node->body_.position.z += 0.001; 
                ik.calculateIK(control_node->feet_, control_node->body_, legs_res);
                control_node->legs_ = *legs_res;
                
                control_node->publishJointStates(control_node->legs_, control_node->head_, &control_node->joint_state_);
                servoDriver.transmitServoPositions(control_node->joint_state_);
                
                rclcpp::spin_some(control_node); 
                rclcpp::sleep_for(10ms);
            }
            control_node->setPrevHexActiveState(true);
        }

        // DURUM: AKTİF YÜRÜYÜŞ
        if (control_node->getHexActiveState() && control_node->getPrevHexActiveState())
        {
            gait.gaitCycle(control_node->cmd_vel_, &control_node->feet_, &control_node->gait_vel_);
            
            // IK fonksiyonu SharedPtr beklediği için legs_res gönderiyoruz
            ik.calculateIK(control_node->feet_, control_node->body_, legs_res);
            control_node->legs_ = *legs_res;
            
            control_node->publishJointStates(control_node->legs_, control_node->head_, &control_node->joint_state_);
            servoDriver.transmitServoPositions(control_node->joint_state_);
            control_node->publishOdometry(control_node->gait_vel_);
            control_node->publishTwist(control_node->gait_vel_);
        }

        // DURUM: OTURMA
        if (!control_node->getHexActiveState() && control_node->getPrevHexActiveState())
        {
            RCLCPP_INFO(control_node->get_logger(), "Hexapod oturuyor...");
            while (control_node->body_.position.z > 0)
            {
                control_node->body_.position.z -= 0.001;
                gait.gaitCycle(control_node->cmd_vel_, &control_node->feet_, &control_node->gait_vel_);
                ik.calculateIK(control_node->feet_, control_node->body_, legs_res);
                control_node->legs_ = *legs_res;
                
                control_node->publishJointStates(control_node->legs_, control_node->head_, &control_node->joint_state_);
                servoDriver.transmitServoPositions(control_node->joint_state_);
                
                rclcpp::spin_some(control_node);
                rclcpp::sleep_for(10ms);
            }
            servoDriver.freeServos();
            control_node->setPrevHexActiveState(false);
        }

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}