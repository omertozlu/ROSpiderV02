#include <ik.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>

// Humble/C++17 için PI tanımını burada yapıyoruz ve bunu kullanıyoruz
static const double PI = 3.14159265358979323846;

//==============================================================================
// Constructor: Parametreleri deklare et ve al
//==============================================================================

Ik::Ik(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
    // ROS 2'de parametre deklarasyonu zorunludur
    node_->declare_parameter("COXA_LENGTH", 0.0);
    node_->declare_parameter("FEMUR_LENGTH", 0.0);
    node_->declare_parameter("TIBIA_LENGTH", 0.0);
    node_->declare_parameter("TARSUS_LENGTH", 0.0);
    node_->declare_parameter("NUMBER_OF_LEGS", 6);
    node_->declare_parameter("COXA_TO_CENTER_X", std::vector<double>{});
    node_->declare_parameter("COXA_TO_CENTER_Y", std::vector<double>{});
    node_->declare_parameter("INIT_FOOT_POS_X", std::vector<double>{});
    node_->declare_parameter("INIT_FOOT_POS_Y", std::vector<double>{});
    node_->declare_parameter("INIT_FOOT_POS_Z", std::vector<double>{});
    node_->declare_parameter("INIT_COXA_ANGLE", std::vector<double>{});

    node_->get_parameter("COXA_LENGTH", COXA_LENGTH);
    node_->get_parameter("FEMUR_LENGTH", FEMUR_LENGTH);
    node_->get_parameter("TIBIA_LENGTH", TIBIA_LENGTH);
    node_->get_parameter("TARSUS_LENGTH", TARSUS_LENGTH);
    node_->get_parameter("NUMBER_OF_LEGS", NUMBER_OF_LEGS);
    
    node_->get_parameter("COXA_TO_CENTER_X", COXA_TO_CENTER_X);
    node_->get_parameter("COXA_TO_CENTER_Y", COXA_TO_CENTER_Y);
    node_->get_parameter("INIT_FOOT_POS_X", INIT_FOOT_POS_X);
    node_->get_parameter("INIT_FOOT_POS_Y", INIT_FOOT_POS_Y);
    node_->get_parameter("INIT_FOOT_POS_Z", INIT_FOOT_POS_Z);
    node_->get_parameter("INIT_COXA_ANGLE", INIT_COXA_ANGLE);
}

Trig Ik::getSinCos(double angle_rad)
{
    return { std::sin(angle_rad), std::cos(angle_rad) };
}

//=============================================================================
// Inverse Kinematics: Matematiksel Hesaplama
//=============================================================================

void Ik::calculateIK(const hexapod_msgs::msg::FeetPositions & feet, 
                     const hexapod_msgs::msg::Pose & body, 
                     hexapod_msgs::msg::LegsJoints::SharedPtr legs)
{
    for (int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++)
    {
        double sign = (leg_index <= 2) ? -1.0 : 1.0;

        Trig A = getSinCos(body.orientation.yaw + feet.foot[leg_index].orientation.yaw);
        Trig B = getSinCos(body.orientation.pitch);
        Trig G = getSinCos(body.orientation.roll);

        double cpr_x = feet.foot[leg_index].position.x + body.position.x - INIT_FOOT_POS_X[leg_index] - COXA_TO_CENTER_X[leg_index];
        double cpr_y = feet.foot[leg_index].position.y + sign * (body.position.y + INIT_FOOT_POS_Y[leg_index] + COXA_TO_CENTER_Y[leg_index]);
        double cpr_z = feet.foot[leg_index].position.z + body.position.z + TARSUS_LENGTH - INIT_FOOT_POS_Z[leg_index];

        double body_pos_x = cpr_x - ((cpr_x * A.cosine * B.cosine) +
                                     (cpr_y * A.cosine * B.sine * G.sine - cpr_y * G.cosine * A.sine) +
                                     (cpr_z * A.sine * G.sine + cpr_z * A.cosine * G.cosine * B.sine));

        double body_pos_y = cpr_y - ((cpr_x * B.cosine * A.sine) +
                                     (cpr_y * A.cosine * G.cosine + cpr_y * A.sine * B.sine * G.sine) +
                                     (cpr_z * G.cosine * A.sine * B.sine - cpr_z * A.cosine * G.sine));

        double body_pos_z = cpr_z - ((-cpr_x * B.sine) + (cpr_y * B.cosine * G.sine) + (cpr_z * B.cosine * G.cosine));

        double feet_pos_x = -INIT_FOOT_POS_X[leg_index] + body.position.x - body_pos_x + feet.foot[leg_index].position.x;
        double feet_pos_y =  INIT_FOOT_POS_Y[leg_index] + sign * (body.position.y - body_pos_y + feet.foot[leg_index].position.y);
        double feet_pos_z =  INIT_FOOT_POS_Z[leg_index] - TARSUS_LENGTH + body.position.z - body_pos_z - feet.foot[leg_index].position.z;

        double femur_to_tarsus = std::sqrt(std::pow(feet_pos_x, 2) + std::pow(feet_pos_y, 2)) - COXA_LENGTH;

        if (std::abs(femur_to_tarsus) > (FEMUR_LENGTH + TIBIA_LENGTH))
        {
            RCLCPP_ERROR(node_->get_logger(), "Ayak %d uzanamiyor!", leg_index);
            continue; 
        }

        double side_a = FEMUR_LENGTH;
        double side_b = TIBIA_LENGTH;
        double side_c = std::sqrt(std::pow(femur_to_tarsus, 2) + std::pow(feet_pos_z, 2));

        // Inverse Kinematics Formulas with Law of Cosines
        double angle_b = std::acos((std::pow(side_a, 2) - std::pow(side_b, 2) + std::pow(side_c, 2)) / (2.0 * side_a * side_c));
        double angle_c = std::acos((std::pow(side_a, 2) + std::pow(side_b, 2) - std::pow(side_c, 2)) / (2.0 * side_a * side_b));

        double theta = std::atan2(femur_to_tarsus, feet_pos_z);

        // Sonuç açıları - Burada yukarida tanimladigimiz PI kullaniliyor
        legs->leg[leg_index].coxa = std::atan2(feet_pos_x, feet_pos_y) + INIT_COXA_ANGLE[leg_index];
        legs->leg[leg_index].femur = (PI / 2.0) - (theta + angle_b);
        legs->leg[leg_index].tibia = (PI / 2.0) - angle_c;
        legs->leg[leg_index].tarsus = legs->leg[leg_index].femur + legs->leg[leg_index].tibia;
    }
}