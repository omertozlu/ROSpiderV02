#include <servo_driver.h>
#include <rclcpp/rclcpp.hpp>

//==============================================================================
// Constructor: Donanım bağlantısı ve Parametre Yönetimi
//==============================================================================

ServoDriver::ServoDriver(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
    // ROS 2'de parametreleri deklare etmeden okumaya çalışmak en büyük hatandır.
    node_->declare_parameter("BAUDRATE", 1000000);
    node_->declare_parameter("DEVICENAME", "/dev/ttyUSB0");
    node_->declare_parameter("TORQUE_ENABLE", 64);
    node_->declare_parameter("GOAL_POSITION_L", 116);
    node_->declare_parameter("PRESENT_POSITION_L", 132);
    node_->declare_parameter("INTERPOLATION_LOOP_RATE", 500);

    // Port ve Paket İşleyicileri (Dynamixel SDK standartları)
    std::string device_name;
    int baudrate;
    node_->get_parameter("BAUDRATE", baudrate);
    node_->get_parameter("DEVICENAME", device_name);

    portHandler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0); // Protocol 1.0/2.0 kontrol et!

    if (portHandler->openPort())
    {
        RCLCPP_INFO(node_->get_logger(), "Port açıldı: %s", device_name.c_str());
        if (portHandler->setBaudRate(baudrate)) 
            RCLCPP_INFO(node_->get_logger(), "Baudrate ayarlandı: %d", baudrate);
        else 
            RCLCPP_ERROR(node_->get_logger(), "Baudrate ayar hatası!");
        portOpenSuccess = true;
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Port açılamadı! Donanım kontrolü yap.");
        portOpenSuccess = false;
    }

    servos_free_ = true;
    
    // Servo Haritasını Okuma (ROS 2'de nested parametreler dikkat ister)
    // Not: Servo isimlerini bir liste olarak alıp detayları prefix ile çekmek en sağlıklısıdır.
    node_->get_parameter("TORQUE_ENABLE", TORQUE_ENABLE);
    node_->get_parameter("INTERPOLATION_LOOP_RATE", INTERPOLATION_LOOP_RATE);
    
    // ... Servo verilerini (id, offset, sign) vektörlere doldurma mantığı ...
    // (Burada Control.cpp'deki gibi declare/get döngüsü kurulmalıdır)
}

ServoDriver::~ServoDriver()
{
    freeServos();
    if (portHandler != nullptr) portHandler->closePort();
}

//==============================================================================
// makeSureServosAreOn: Torque Yönetimi
//==============================================================================

void ServoDriver::makeSureServosAreOn(const sensor_msgs::msg::JointState & joint_state)
{
    if (!servos_free_ || !portOpenSuccess) return;

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        uint16_t dxl_present_pos = 0;
        if (packetHandler->read2ByteTxRx(portHandler, ID[i], PRESENT_POSITION_L, &dxl_present_pos, &dxl_error) == COMM_SUCCESS)
        {
            cur_pos_[i] = dxl_present_pos;
        }
    }

    rclcpp::sleep_for(std::chrono::milliseconds(100));

    bool all_torque_on = true;
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        if (packetHandler->write1ByteTxRx(portHandler, ID[i], TORQUE_ENABLE, 1, &dxl_error) != COMM_SUCCESS)
        {
            RCLCPP_WARN(node_->get_logger(), "Torque hatası ID: %d", ID[i]);
            all_torque_on = false;
        }
    }

    if (all_torque_on)
    {
        RCLCPP_INFO(node_->get_logger(), "Hexapod torku aktif.");
        servos_free_ = false;
    }
}

//==============================================================================
// transmitServoPositions: Interpolasyon ve Veri Gönderimi
//==============================================================================

void ServoDriver::transmitServoPositions(const sensor_msgs::msg::JointState & joint_state)
{
    if (!portOpenSuccess) return;

    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, GOAL_POSITION_L, 2);
    convertAngles(joint_state);
    makeSureServosAreOn(joint_state);

    int interpolating = 0;
    std::vector<int> complete(SERVO_COUNT, 0);

    for (int i = 0; i < SERVO_COUNT; i++)
    {
        if (std::abs(cur_pos_[i] - goal_pos_[i]) > 2) // Ölü bölge (Deadzone) ekledik
        {
            interpolating++;
            write_pos_[i] = cur_pos_[i];
        }
        else
        {
            write_pos_[i] = goal_pos_[i];
            complete[i] = 1;
        }
    }

    rclcpp::Rate loop_rate(INTERPOLATION_LOOP_RATE);
    
    while (interpolating > 0 && rclcpp::ok())
    {
        for (int i = 0; i < SERVO_COUNT; i++)
        {
            if (complete[i] == 0)
            {
                // Basit lineer interpolasyon
                int step = (cur_pos_[i] < goal_pos_[i]) ? 1 : -1;
                write_pos_[i] += step;

                if ((step > 0 && write_pos_[i] >= goal_pos_[i]) || (step < 0 && write_pos_[i] <= goal_pos_[i]))
                {
                    write_pos_[i] = goal_pos_[i];
                    complete[i] = 1;
                    interpolating--;
                }
            }
            
            uint8_t param_goal_pos[2];
            param_goal_pos[0] = DXL_LOBYTE(write_pos_[i]);
            param_goal_pos[1] = DXL_HIBYTE(write_pos_[i]);
            groupSyncWrite.addParam(ID[i], param_goal_pos);
        }

        groupSyncWrite.txPacket();
        groupSyncWrite.clearParam();
        loop_rate.sleep();
    }

    for (int i = 0; i < SERVO_COUNT; i++) cur_pos_[i] = write_pos_[i];
}

void ServoDriver::freeServos()
{
    if (!portOpenSuccess) return;
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        packetHandler->write1ByteTxRx(portHandler, ID[i], TORQUE_ENABLE, 0, &dxl_error);
    }
    servos_free_ = true;
    RCLCPP_INFO(node_->get_logger(), "Tork kapatıldı.");
}

// src/servo_driver.cpp icine ekle
void ServoDriver::convertAngles(const sensor_msgs::msg::JointState & joint_state)
{
    for (int i = 0; i < SERVO_COUNT; i++)
    {
        // Formül: goal = center + round((angle - offset) * resolution)
        // Offset ve sign (servo_orientation) donanim konfigürasyonuna göredir.
        goal_pos_[i] = CENTER[i] + std::round((joint_state.position[i] - (servo_orientation_[i] * OFFSET[i])) * RAD_TO_SERVO_RESOLUTION[i]);
    }
}