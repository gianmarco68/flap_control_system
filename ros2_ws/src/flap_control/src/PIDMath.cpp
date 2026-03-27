// ros2_ws/src/flap_control/src/PIDMath.cpp

#include "flap_control/PIDMath.hpp"
#include <algorithm>
#include <cmath>

namespace flap_control {

    PIDMath::PIDMath(double kp, double ki, double kd, double dt,
                    double output_min, double output_max, double cutoff_freq)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt),
        output_min_(output_min), output_max_(output_max)
    {
        calculateFilterCoefficients(cutoff_freq);
    }

    double PIDMath::compute(double setpoint, double measurement) {
        double error = setpoint - measurement;

        // 1. Termine Proporzionale
        last_P_ = kp_ * error;
        
        // 2. Termine Derivativo con filtro passa-basso

        // Calcolo la vera velocità di variazione (Derivata grezza)
        // modificare derivativo (calcolare sull'uscita e non sull'errore)
        double raw_derivative = -(measurement - prev_measurement_) / dt_;
        // Applico il filtro passa-basso alla derivata (non all'errore)
        double filtered_derivative = filter_error(raw_derivative);
        // Calcolo l'output del freno
        last_D_ = kd_ * filtered_derivative;

        // 3. Termine Integrale con Anti-Windup (Clamping)
        integral_ += error * dt_;
        double I = ki_ * integral_;
        
        // Calcolo dei limiti dinamici per l'integrale
        double max_I = output_max_ - last_P_ - last_D_;
        double min_I = output_min_ - last_P_ - last_D_;
        I = std::clamp(I, min_I, max_I);
        
        // Ricalcolo dell'integrale interno se stiamo saturando
        if (ki_ != 0.0) {
            integral_ = I / ki_;
        }

        

        // 4. Somma totale e Clamp finale
        last_output_ = last_P_ + I + last_D_;
        last_output_ = std::clamp(last_output_, output_min_, output_max_);
        
        // Aggiornamento stato
        prev_measurement_ = measurement;
        prev_derivative_ = filtered_derivative;

        return last_output_;
    }

    double PIDMath::filter_error(double raw_err) {
        // Inizializzazione al primo ciclo
        if (!prev_err_set_) {
            prev_err_filtered_err_ = raw_err;
            prev_raw_err_ = raw_err;
            prev_err_set_ = true;
            return raw_err;
        }

        // Applicazione del filtro
        double err_filtered = kf_ * prev_err_filtered_err_ + kr_ * (raw_err + prev_raw_err_);

        // Aggiornamento memorie del filtro
        prev_err_filtered_err_ = err_filtered;
        prev_raw_err_ = raw_err;

        return err_filtered;
    }

    void PIDMath::updateCoefficients(double kp, double ki, double kd,
                                    double output_min, double output_max, double cutoff_freq) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        output_min_ = output_min;
        output_max_ = output_max;
        calculateFilterCoefficients(cutoff_freq);
    }

    void PIDMath::updateCoefficients(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    void PIDMath::calculateFilterCoefficients(double cutoff_freq) {
        // Trasformata bilineare (Tustin) per filtro passa basso del 1° ordine
        double denom = M_PI * cutoff_freq * dt_ + 1.0;
        kf_ = - (M_PI * cutoff_freq * dt_ - 1.0) / denom;
        kr_ =   (M_PI * cutoff_freq * dt_) / denom;
    }

    void PIDMath::reset() {
        prev_measurement_ = 0.0;
        integral_ = 0.0;
        prev_derivative_ = 0.0;
        
        prev_err_filtered_err_ = 0.0;
        prev_raw_err_ = 0.0;
        prev_err_set_ = false;

        last_P_ = 0.0;
        last_D_ = 0.0;
        last_output_ = 0.0;
    }

} // namespace flap_control