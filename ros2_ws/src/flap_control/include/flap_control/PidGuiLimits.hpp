// ros2_ws/src/flap_control/include/flap_control/PidGuiLimits.hpp
#pragma once

namespace flap_control {

  struct PidGuiLimits {
    double kp_min;
    double kp_max;
    double kp_step;

    double ki_min;
    double ki_max;
    double ki_step;

    double kd_min;
    double kd_max;
    double kd_step;

    double height_min;
    double height_max;
    double height_step;

    double flap_angle_limit_min;
    double flap_angle_limit_max;
    double flap_step;
  };

} // namespace flap_control

// // Default “di progetto”
// inline PidGuiLimits default_pid_gui_limits()
// {
//   PidGuiLimits lim{};

//   lim.kp_min   = 0.0;
//   lim.kp_max   = 20.0;
//   lim.kp_step  = 0.1;

//   lim.ki_min   = 0.0;
//   lim.ki_max   = 5.0;
//   lim.ki_step  = 0.01;

//   lim.kd_min   = 0.0;
//   lim.kd_max   = 5.0;
//   lim.kd_step  = 0.01;

//   // Qui scegli tu con il team i numeri “ragionevoli”
//   lim.height_min   = 0.0;
//   lim.height_max   = 1.5;
//   lim.height_step  = 0.05;  // 5 cm

//   lim.flap_angle_limit_min     = -30.0;
//   lim.flap_angle_limit_max     =  30.0;
//   lim.flap_step    =   1.0; // 1 grado

//   return lim;
// }
