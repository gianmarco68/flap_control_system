// ros2_ws/src/flap_control/include/flap_control/ConfigManager.hpp
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "flap_control/PidGuiLimits.hpp" // Ricicliamo la struct che hai già!

namespace flap_control {

// Raggruppiamo tutte le condizioni iniziali in una singola struct pulita
struct InitialConditions {
    double dt;
    double feedback;
    double kp;
    double ki;
    double kd;
    double flap_angle_limit_min;
    double flap_angle_limit_max;
    double cutoff_freq;
    double height_target;
    double current_sog;
    double current_roll;
    double current_pitch;
    double sog_threshold;
    double roll_max;
    double pitch_max;
    double neutral_flap_angle;
    double gear_ratio;
    double servo_offset;
};

class ConfigManager {
    public:
        ConfigManager() = default;

        // L'unica funzione che il nodo chiamerà per caricare tutto
        void init(rclcpp::Node* node);

        // Getters per accedere ai parametri in modo sicuro
        const InitialConditions& getInitialConditions() const { return init_cond_; }
        const PidGuiLimits& getGuiLimits() const { return gui_limits_; }

    private:
        InitialConditions init_cond_;
        PidGuiLimits gui_limits_;
    };

} // namespace flap_control