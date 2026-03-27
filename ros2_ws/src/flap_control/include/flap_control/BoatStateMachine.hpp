// ros2_ws/src/flap_control/include/flap_control/BoatStateMachine.hpp
#pragma once

namespace flap_control {

// La nostra macchina a stati
enum class ControllerState {
    OFF, // Il PID è spento, flap neutro
    ON   // Il PID lavora e calcola l'angolo del flap
};

class BoatStateMachine {
    public:
        // Inizializza la macchina con le soglie prese dal ConfigManager
        BoatStateMachine(double sog_threshold, double roll_max, double pitch_max);

        // Aggiorna le letture dai sensori
        void updateSensors(double sog, double roll, double pitch);

        // Aggiorna le soglie a runtime
        void updateThresholds(double sog_threshold, double roll_max, double pitch_max);

        // Imposta quanto tempo (in secondi) tollerare dati assenti/errati prima di spegnere
        void setTimeoutDuration(double timeout_sec) { timeout_duration_ = timeout_sec; }

        // Valuta la logica. Passiamo il dt (es. 0.1s) per farle tenere il tempo!
        bool evaluate(double dt);

        // Getter per sapere lo stato attuale
        ControllerState getState() const { return current_state_; }

    private:
        ControllerState current_state_{ControllerState::OFF};

        // Soglie
        double sog_threshold_;
        double roll_max_;
        double pitch_max_;

        // Memoria per il Timeout (Il Fix Magico)
        double time_since_invalid_{0.0};
        double timeout_duration_{0.5}; // Default: tollera fino a mezzo secondo di problemi

        // Valori attuali
        double current_sog_{0.0};
        double current_roll_{0.0};
        double current_pitch_{0.0};
    };

} // namespace flap_control