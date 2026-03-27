// ros2_ws/src/flap_control/include/flap_control/PIDMath.hpp
#pragma once

namespace flap_control {

class PIDMath {
    public:
        PIDMath(double kp, double ki, double kd, double dt,
                double output_min, double output_max, double cutoff_freq);

        // I due metodi di calcolo principali
        double compute(double setpoint, double measurement);

        // Aggiornamento dei parametri a runtime
        void updateCoefficients(double kp, double ki, double kd,
                                double output_min, double output_max, double cutoff_freq);
        void updateCoefficients(double kp, double ki, double kd);

        // Reset dello stato interno (integrale, derivata, ecc.)
        void reset();

        // Getters per leggere lo stato interno (utili per il logging nel nodo ROS)
        double getP() const { return last_P_; }
        double getI() const { return integral_; } // L'integrale scalato per ki è il termine I
        double getD() const { return last_D_; }
        double getError() const { return last_error_; }
        double getOutput() const { return last_output_; }

    private:
        double filter_error(double raw_err);
        void calculateFilterCoefficients(double cutoff_freq);

        // Guadagni PID
        double kp_;
        double ki_;
        double kd_;

        // Tempo di campionamento e limiti
        double dt_;
        double output_min_;
        double output_max_;

        // Coefficienti filtro passa-basso per la derivata
        double kf_;
        double kr_;
        double prev_err_filtered_err_{0.0};
        double prev_raw_err_{0.0};
        bool prev_err_set_{false};

        // Variabili di stato interne
        double prev_measurement_{0.0};
        double integral_{0.0};
        double prev_derivative_{0.0};
        double last_error_{0.0};

        // Salvataggio dell'ultimo calcolo per i log
        double last_P_{0.0};
        double last_D_{0.0};
        double last_output_{0.0};
    };

} // namespace flap_control