// AUTO-GENERATED FILE. DO NOT EDIT.
#ifndef FLAP_CONTROL_ROS_ENDPOINTS_HPP
#define FLAP_CONTROL_ROS_ENDPOINTS_HPP

namespace ros_endpoints {

struct SharedTopics {
  static constexpr const char* CURRENT_HEIGHT_EST = "/current_height_est";
  static constexpr const char* GPS_VEL_DATA = "/gps_vel_data";
  static constexpr const char* IMU_DATA = "/imu_data";
  static constexpr const char* WAND_ANGLE = "/wand_angle";
  static constexpr const char* LOCAL_CMD_UPDATE = "/local_cmd/update";
  static constexpr const char* SHOW_SERVO_ANGLE_CMD = "/processed/manual_servo_cmd";
  static constexpr const char* CAN_COMMAND = "/can_command";
  static constexpr const char* PID_INDICATORS = "/pid_indicators";
  static constexpr const char* CONTROL_VALUE = "/control_value";
};

struct SharedServices {
};

using Shared = SharedTopics;

struct PID_CONTROLLER_NODE {
  struct SUB {
    static constexpr const char* CURRENT_HEIGHT_EST = "/current_height_est";
    static constexpr const char* GPS_VEL_DATA = "/gps_vel_data";
    static constexpr const char* IMU_DATA = "/imu_data";
    static constexpr const char* LOCAL_CMD_UPDATE = "/local_cmd/update";
    static constexpr const char* SHOW_SERVO_ANGLE_CMD = "/processed/manual_servo_cmd";
  };
  struct PUB {
    static constexpr const char* PID_INDICATORS = "/pid_indicators";
    static constexpr const char* CONTROL_VALUE = "/control_value";
  };
};

struct FLAP_CONTROLLER_NODE {
  struct SUB {
    static constexpr const char* CONTROL_VALUE = "/control_value";
  };
  struct PUB {
    static constexpr const char* CAN_COMMAND = "/can_command";
  };
};

} // namespace ros_endpoints

#endif // FLAP_CONTROL_ROS_ENDPOINTS_HPP
