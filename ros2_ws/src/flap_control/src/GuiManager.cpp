// ros2_ws/src/flap_control/src/GuiManager.cpp
#include "flap_control/GuiManager.hpp"
#include <algorithm>

namespace flap_control {

GuiManager::GuiManager(const ConfigManager& config_manager)
    : config_manager_(config_manager) {}

void GuiManager::applyStep(double& value, const std::string& method, double step, double min, double max) {
    if (method == "INCREASE") {
        value = std::min(value + step, max);
    } else if (method == "DECREASE") {
        value = std::max(value - step, min);
    }
}

bool GuiManager::processUpdate(const sail_msgs::msg::Update::SharedPtr msg,
                               double& kp, double& ki, double& kd, 
                               double& setpoint, 
                               double& limit_min, double& limit_max) {
    const std::string& indicator = msg->indicator;
    const std::string& method = msg->method;
    auto limits = config_manager_.getGuiLimits();
    bool changed = false;

    // Salviamo lo stato precedente per verificare se c'è stato un VERO cambiamento
    double old_kp = kp, old_ki = ki, old_kd = kd, old_sp = setpoint, old_lmin = limit_min, old_lmax = limit_max;

    if (indicator == "KP") applyStep(kp, method, limits.kp_step, limits.kp_min, limits.kp_max);
    else if (indicator == "KI") applyStep(ki, method, limits.ki_step, limits.ki_min, limits.ki_max);
    else if (indicator == "KD") applyStep(kd, method, limits.kd_step, limits.kd_min, limits.kd_max);
    else if (indicator == "HEIGHT_TARGET") applyStep(setpoint, method, limits.height_step, limits.height_min, limits.height_max);
    else if (indicator == "FLAP_ANGLE_BOUNDARY_MAX") applyStep(limit_max, method, limits.flap_step, limit_min, limits.flap_angle_limit_max);
    else if (indicator == "FLAP_ANGLE_BOUNDARY_MIN") applyStep(limit_min, method, limits.flap_step, limits.flap_angle_limit_min, limit_max);

    // Controlliamo se qualche valore è effettivamente cambiato
    if (kp != old_kp || ki != old_ki || kd != old_kd || 
        setpoint != old_sp || limit_min != old_lmin || limit_max != old_lmax) {
        changed = true;
    }

    return changed;
}

sail_msgs::msg::IndicatorState GuiManager::makeState(const std::string& name, double value, double vmin, double vmax) const {
    sail_msgs::msg::IndicatorState s;
    s.indicator = name;
    s.value = static_cast<float>(value);
    s.failed = false; 
    s.can_increase = value < (vmax - 1e-6);
    s.can_decrease = value > (vmin + 1e-6);
    return s;
}

sail_msgs::msg::Indicators GuiManager::generateIndicatorsMessage(
                               double kp, double ki, double kd, 
                               double setpoint, 
                               double limit_min, double limit_max) const {
    auto lim = config_manager_.getGuiLimits();
    sail_msgs::msg::Indicators msg;

    msg.indicators.push_back(makeState("KP", kp, lim.kp_min, lim.kp_max));
    msg.indicators.push_back(makeState("KI", ki, lim.ki_min, lim.ki_max));
    msg.indicators.push_back(makeState("KD", kd, lim.kd_min, lim.kd_max));
    msg.indicators.push_back(makeState("HEIGHT_TARGET", setpoint, lim.height_min, lim.height_max));
    msg.indicators.push_back(makeState("FLAP_ANGLE_BOUNDARY_MAX", limit_max, limit_min, lim.flap_angle_limit_max));
    msg.indicators.push_back(makeState("FLAP_ANGLE_BOUNDARY_MIN", limit_min, lim.flap_angle_limit_min, limit_max));

    return msg;
}

} // namespace flap_control