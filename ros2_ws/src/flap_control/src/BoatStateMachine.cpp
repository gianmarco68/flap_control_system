// ros2_ws/src/flap_control/src/BoatStateMachine.cpp

#include "flap_control/BoatStateMachine.hpp"
#include <cmath> // Per std::abs

namespace flap_control {

BoatStateMachine::BoatStateMachine(double sog_threshold, double roll_max, double pitch_max)
    : sog_threshold_(sog_threshold), roll_max_(roll_max), pitch_max_(pitch_max) 
{
}

void BoatStateMachine::updateSensors(double sog, double roll, double pitch) {
    current_sog_ = sog;
    current_roll_ = roll;
    current_pitch_ = pitch;
}

void BoatStateMachine::updateThresholds(double sog_threshold, double roll_max, double pitch_max) {
    sog_threshold_ = sog_threshold;
    roll_max_ = roll_max;
    pitch_max_ = pitch_max;
}

bool BoatStateMachine::evaluate(double dt) {
    // 1. Valutazione delle singole condizioni (come prima)
    bool is_fast_enough = current_sog_ > sog_threshold_;
    bool is_stable_roll = std::abs(current_roll_) < roll_max_;
    bool is_stable_pitch = std::abs(current_pitch_) < pitch_max_;

    // 2. Condizione globale istantanea
    bool conditions_met = is_fast_enough && is_stable_roll && is_stable_pitch;

    // Partiamo dal presupposto di mantenere lo stato attuale
    ControllerState next_state = current_state_;

    // 3. Logica con Memoria (Timeout)
    if (conditions_met) {
        // Tutto perfetto: azzeriamo il timer di panico e accendiamo il PID
        time_since_invalid_ = 0.0;
        next_state = ControllerState::ON;
    } else {
        // C'è un problema. Incrementiamo il timer usando il dt
        time_since_invalid_ += dt;
        
        // Spegniamo solo se il problema persiste oltre la tolleranza!
        if (time_since_invalid_ >= timeout_duration_) {
            next_state = ControllerState::OFF;
        }
    }

    // 4. Controllo se c'è stata una transizione (per stamparlo nei log)
    if (next_state != current_state_) {
        current_state_ = next_state;
        return true; 
    }

    return false;
}

} // namespace flap_control