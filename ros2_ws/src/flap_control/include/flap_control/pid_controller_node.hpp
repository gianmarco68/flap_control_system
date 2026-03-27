// ros2_ws/src/flap_control/include/flap_control/pid_controller_node.hpp
#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "sail_msgs/msg/indicators.hpp"
#include "sail_msgs/msg/indicator_state.hpp"
#include "sail_msgs/msg/update.hpp"
#include "sail_msgs/msg/show_limit_servo.hpp"

// I nostri mattoncini
#include "flap_control/ConfigManager.hpp"
#include "flap_control/BoatStateMachine.hpp"
#include "flap_control/PIDMath.hpp"
#include "flap_control/GuiManager.hpp" // <-- AGGIUNTO

namespace flap_control {

class PIDControllerNode : public rclcpp::Node {
public:
    PIDControllerNode();

private:
    // ==== Metodi Interni ====
    void publishIndicators();
    void updateFromGui(const sail_msgs::msg::Update::SharedPtr msg);
    void timerCallback();
    double flapToServoAngle(double flap_angle);

    // ==== Callbacks Sensori ====
    void heightCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void speedCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
    void resetCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void showLimitServoCallback(const sail_msgs::msg::ShowLimitServo::SharedPtr msg);

    // ==== Membri ROS ====
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr feedback_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sog_sub_;
    rclcpp::Subscription<sail_msgs::msg::Update>::SharedPtr update_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_pub_;
    rclcpp::Publisher<sail_msgs::msg::Indicators>::SharedPtr indicators_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::Subscription<sail_msgs::msg::ShowLimitServo>::SharedPtr show_servo_limit_sub_;
    // ==== I nostri moduli core ====
    ConfigManager config_manager_;
    std::unique_ptr<BoatStateMachine> state_machine_;
    std::unique_ptr<PIDMath> pid_math_;
    std::unique_ptr<GuiManager> gui_manager_; // <-- AGGIUNTO

    // ==== Variabili di lavoro correnti ====
    double kp_, ki_, kd_;
    double flap_angle_limit_min_, flap_angle_limit_max_;
    double cutoff_freq_;
    double setpoint_;
    double feedback_{0.0};

    // Sensori grezzi
    double current_sog_{0.0};  
    double current_roll_{0.0};   
    double current_pitch_{0.0};  

    // Cinematica e costanti
    double neutral_flap_angle_;    
    double gear_ratio_;
    double servo_offset_;

    double dt_;

    // Variabili per override manuale del servo
    bool override_active_{false};
    rclcpp::Time override_end_time_;
    uint8_t override_type_{0}; // 0 = SERVO_MIN, 1 = SERVO_MAX
};

} // namespace flap_control