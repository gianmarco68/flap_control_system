// ros2_ws/src/flap_control/src/ConfigManager.cpp

#include "flap_control/ConfigManager.hpp"

namespace flap_control {

void ConfigManager::init(rclcpp::Node* node) {
    // 1. DICHIARAZIONE PARAMETRI
    // --- initial_conditions ---
    node->declare_parameter<double>("initial_conditions.dt", 0.1);
    node->declare_parameter<double>("initial_conditions.feedback", 0.0);
    node->declare_parameter<double>("initial_conditions.kp", 1.0);
    node->declare_parameter<double>("initial_conditions.ki", 0.0);
    node->declare_parameter<double>("initial_conditions.kd", 0.0);
    node->declare_parameter<double>("initial_conditions.flap_angle_limit_min", -15.0);
    node->declare_parameter<double>("initial_conditions.flap_angle_limit_max", 15.0);
    node->declare_parameter<double>("initial_conditions.cutoff_freq", 0.5);
    node->declare_parameter<double>("initial_conditions.height_target", 0.5);
    node->declare_parameter<double>("initial_conditions.current_sog", 0.0);
    node->declare_parameter<double>("initial_conditions.current_roll", 0.0);
    node->declare_parameter<double>("initial_conditions.current_pitch", 0.0);
    node->declare_parameter<double>("initial_conditions.sog_threshold", 5.0);
    node->declare_parameter<double>("initial_conditions.roll_max", 70.0);
    node->declare_parameter<double>("initial_conditions.pitch_max", 45.0);
    node->declare_parameter<double>("initial_conditions.neutral_flap_angle", 0.0);
    node->declare_parameter<double>("initial_conditions.gear_ratio", 1.0);
    node->declare_parameter<double>("initial_conditions.servo_offset", 0.0);

    // --- GUI_limits ---
    node->declare_parameter<double>("GUI_limits.kp_min", 0.0);
    node->declare_parameter<double>("GUI_limits.kp_max", 10.0);
    node->declare_parameter<double>("GUI_limits.kp_step", 0.1);
    node->declare_parameter<double>("GUI_limits.ki_min", 0.0);
    node->declare_parameter<double>("GUI_limits.ki_max", 10.0);
    node->declare_parameter<double>("GUI_limits.ki_step", 0.1);
    node->declare_parameter<double>("GUI_limits.kd_min", 0.0);
    node->declare_parameter<double>("GUI_limits.kd_max", 10.0);
    node->declare_parameter<double>("GUI_limits.kd_step", 0.1);
    node->declare_parameter<double>("GUI_limits.height_min", 0.0);
    node->declare_parameter<double>("GUI_limits.height_max", 1.0);
    node->declare_parameter<double>("GUI_limits.height_step", 0.01);
    node->declare_parameter<double>("GUI_limits.flap_angle_limit_min", -15.0);
    node->declare_parameter<double>("GUI_limits.flap_angle_limit_max", 15.0);
    node->declare_parameter<double>("GUI_limits.flap_step", 1.0);

    // 2. LETTURA DAL YAML E ASSEGNAZIONE
    init_cond_.dt = node->get_parameter("initial_conditions.dt").as_double();
    init_cond_.feedback = node->get_parameter("initial_conditions.feedback").as_double();
    init_cond_.kp = node->get_parameter("initial_conditions.kp").as_double();
    init_cond_.ki = node->get_parameter("initial_conditions.ki").as_double();
    init_cond_.kd = node->get_parameter("initial_conditions.kd").as_double();
    init_cond_.flap_angle_limit_min = node->get_parameter("initial_conditions.flap_angle_limit_min").as_double();
    init_cond_.flap_angle_limit_max = node->get_parameter("initial_conditions.flap_angle_limit_max").as_double();
    init_cond_.cutoff_freq = node->get_parameter("initial_conditions.cutoff_freq").as_double();
    init_cond_.height_target = node->get_parameter("initial_conditions.height_target").as_double();
    init_cond_.current_sog = node->get_parameter("initial_conditions.current_sog").as_double();
    init_cond_.current_roll = node->get_parameter("initial_conditions.current_roll").as_double();
    init_cond_.current_pitch = node->get_parameter("initial_conditions.current_pitch").as_double();
    init_cond_.sog_threshold = node->get_parameter("initial_conditions.sog_threshold").as_double();
    init_cond_.roll_max = node->get_parameter("initial_conditions.roll_max").as_double();
    init_cond_.pitch_max = node->get_parameter("initial_conditions.pitch_max").as_double();
    init_cond_.neutral_flap_angle = node->get_parameter("initial_conditions.neutral_flap_angle").as_double();
    init_cond_.gear_ratio = node->get_parameter("initial_conditions.gear_ratio").as_double();
    init_cond_.servo_offset = node->get_parameter("initial_conditions.servo_offset").as_double();

    gui_limits_.kp_min = node->get_parameter("GUI_limits.kp_min").as_double();
    gui_limits_.kp_max = node->get_parameter("GUI_limits.kp_max").as_double();
    gui_limits_.kp_step = node->get_parameter("GUI_limits.kp_step").as_double();
    gui_limits_.ki_min = node->get_parameter("GUI_limits.ki_min").as_double();
    gui_limits_.ki_max = node->get_parameter("GUI_limits.ki_max").as_double();
    gui_limits_.ki_step = node->get_parameter("GUI_limits.ki_step").as_double();
    gui_limits_.kd_min = node->get_parameter("GUI_limits.kd_min").as_double();
    gui_limits_.kd_max = node->get_parameter("GUI_limits.kd_max").as_double();
    gui_limits_.kd_step = node->get_parameter("GUI_limits.kd_step").as_double();
    gui_limits_.height_min = node->get_parameter("GUI_limits.height_min").as_double();
    gui_limits_.height_max = node->get_parameter("GUI_limits.height_max").as_double();
    gui_limits_.height_step = node->get_parameter("GUI_limits.height_step").as_double();
    gui_limits_.flap_angle_limit_min = node->get_parameter("GUI_limits.flap_angle_limit_min").as_double();
    gui_limits_.flap_angle_limit_max = node->get_parameter("GUI_limits.flap_angle_limit_max").as_double();
    gui_limits_.flap_step = node->get_parameter("GUI_limits.flap_step").as_double();

    RCLCPP_INFO(node->get_logger(), "ConfigManager: Parametri caricati con successo dal file YAML.");
}

} // namespace flap_control