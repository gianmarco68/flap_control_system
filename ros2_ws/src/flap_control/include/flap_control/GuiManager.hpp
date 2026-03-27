// ros2_ws/src/flap_control/include/flap_control/GuiManager.hpp
#pragma once

#include <string>
#include <vector>
#include "sail_msgs/msg/indicators.hpp"
#include "sail_msgs/msg/indicator_state.hpp"
#include "sail_msgs/msg/update.hpp"
#include "flap_control/ConfigManager.hpp" // Per leggere i limiti

namespace flap_control {

class GuiManager {
public:
    GuiManager(const ConfigManager& config_manager);

    // Riceve il messaggio e modifica i valori per riferimento.
    // Ritorna 'true' se un valore è stato effettivamente modificato.
    bool processUpdate(const sail_msgs::msg::Update::SharedPtr msg,
                       double& kp, double& ki, double& kd, 
                       double& setpoint, 
                       double& limit_min, double& limit_max);

    // Genera il messaggio con tutti gli indicatori pronti per essere pubblicati
    sail_msgs::msg::Indicators generateIndicatorsMessage(
                       double kp, double ki, double kd, 
                       double setpoint, 
                       double limit_min, double limit_max) const;

private:
    const ConfigManager& config_manager_;

    // Helper per l'incremento/decremento generico
    void applyStep(double& value, const std::string& method, double step, double min, double max);

    // Helper per generare un singolo stato
    sail_msgs::msg::IndicatorState makeState(const std::string& name, double value, double vmin, double vmax) const;
};

} // namespace flap_control