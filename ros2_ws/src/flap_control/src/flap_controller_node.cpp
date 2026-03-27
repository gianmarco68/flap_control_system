// ros2_ws/src/flap_control/src/flap_controller_node.cpp
/*
 * ======================================================================================
 * NODO: FLAP CONTROLLER NODE
 * ======================================================================================
 * * DESCRIZIONE:
 * Legge un angolo RELATIVO in GRADI (es. -20 a +20) da /control_value o /wand_angle
 * Lo traduce in un angolo ASSOLUTO (es. 70 a 110) sommando il neutro hardware (90).
 * Lo converte in un pacchetto CAN a 16-bit (0-65535) per il motore.
 * ======================================================================================
 */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp" 
#include "sail_msgs/msg/can_cmd.hpp" 
#include "flap_control/ros_endpoints.hpp"
#include "flap_control/config_flap_controller_node.hpp"

#include <algorithm>
#include <cmath>

using std::placeholders::_1;

class FlapControllerNode : public rclcpp::Node
{
public:
    FlapControllerNode() : Node("flap_controller_node")
    {
        control_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            ros_endpoints::FLAP_CONTROLLER_NODE::SUB::CONTROL_VALUE, 10, 
            std::bind(&FlapControllerNode::control_callback, this, _1)
        );

        can_pub_ = this->create_publisher<sail_msgs::msg::CanCmd>(
            ros_endpoints::FLAP_CONTROLLER_NODE::PUB::CAN_COMMAND, 10
        );

        RCLCPP_INFO(this->get_logger(), "Flap Controller Started. Listening for relative angles (-/+)...");
    }

private:
    void control_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // 1. Lettura Angolo Relativo (es. -15.0)
        double relative_angle_deg = static_cast<double>(msg->data);

        // 2. TRADUZIONE IN ANGOLO ASSOLUTO HARDWARE
        double hardware_neutral_deg = 90.0;
        double absolute_angle_deg = hardware_neutral_deg + relative_angle_deg;

        // 3. Clamp Fisico (Sicurezza Estrema)
        absolute_angle_deg = std::clamp(absolute_angle_deg, 0.0, 180.0);

        // --- MODIFICA FONDAMENTALE QUI ---
        // Invece di fare proporzioni strane con 65535, arrotondiamo l'angolo
        // direttamente al numero intero più vicino (es. 90.5 gradi diventa 91).
        uint16_t angle_uint16 = static_cast<uint16_t>(std::round(absolute_angle_deg));
        // ---------------------------------

        // 5. COSTRUZIONE PACCHETTO CAN
        sail_msgs::msg::CanCmd cmd_msg;
        cmd_msg.can_id = FlapConfig::MOTOR_CAN_ID;
        cmd_msg.payload.data.resize(8);

        cmd_msg.payload.data[0] = FlapConfig::MANUF_CODE_MSB; 
        cmd_msg.payload.data[1] = FlapConfig::MANUF_CODE_LSB; 

        // E ora dividiamo il nostro intero (es. 91) nei due byte richiesti:
        // data[2] è LSB (Byte meno significativo)
        cmd_msg.payload.data[2] = static_cast<uint8_t>(angle_uint16 & 0xFF);
        // data[3] è MSB (Byte più significativo)
        cmd_msg.payload.data[3] = static_cast<uint8_t>((angle_uint16 >> 8) & 0xFF);

        // Il resto dei byte rimane uguale
        cmd_msg.payload.data[4] = static_cast<uint8_t>(angle_uint16 & 0xFF); // Stesso angolo se richiesto
        cmd_msg.payload.data[5] = static_cast<uint8_t>((angle_uint16 >> 8) & 0xFF);
        cmd_msg.payload.data[6] = 0xFF;
        cmd_msg.payload.data[7] = 0xFF;

        // 6. INVIO
        can_pub_->publish(cmd_msg);

        // LOG utile: mostra sia il relativo (per debug GUI) che l'assoluto (per debug hardware)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), FlapConfig::LOG_THROTTLE_MS, 
            "Rel: %+.1f deg -> Abs: %.1f deg -> CAN: %d", relative_angle_deg, absolute_angle_deg, angle_uint16);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr control_sub_;
    rclcpp::Publisher<sail_msgs::msg::CanCmd>::SharedPtr can_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FlapControllerNode>());
    rclcpp::shutdown();
    return 0;
}