// AUTO-GENERATED FILE. DO NOT EDIT.
#ifndef STATE_ESTIMATION_ROS_ENDPOINTS_HPP
#define STATE_ESTIMATION_ROS_ENDPOINTS_HPP

namespace ros_endpoints {

struct SharedTopics {
  static constexpr const char* GPS_DATA = "/gps_data";
  static constexpr const char* IMU_DATA = "/imu_data";
  static constexpr const char* WAND_ANGLE = "/wand_angle";
  static constexpr const char* GPS_DATA_FIXED = "/gps/data_fixed";
  static constexpr const char* GPS_ODOM_LOCAL = "/gps/odom_local";
  static constexpr const char* FILTERED_ODOM = "/odometry/filtered";
  static constexpr const char* CURRENT_HEIGHT_EST = "/current_height_est";
  static constexpr const char* GPS_FILTERED = "/gps/filtered";
};

struct SharedServices {
};

using Shared = SharedTopics;

struct HEIGHT_ESTIMATOR_NODE {
  struct SUB {
    static constexpr const char* WAND_ANGLE = "/wand_angle";
    static constexpr const char* IMU_DATA = "/imu_data";
  };
  struct PUB {
    static constexpr const char* CURRENT_HEIGHT_EST = "/current_height_est";
  };
};

struct GPS_HEADER_FIXER_NODE {
  struct SUB {
    static constexpr const char* GPS_DATA = "/gps_data";
  };
  struct PUB {
    static constexpr const char* GPS_DATA_FIXED = "/gps/data_fixed";
  };
};

struct GPS_INPUT_ADAPTER {
  struct SUB {
    static constexpr const char* GPS_DATA_FIXED = "/gps/data_fixed";
  };
  struct PUB {
    static constexpr const char* GPS_ODOM_LOCAL = "/gps/odom_local";
  };
};

struct GPS_OUTPUT_ADAPTER {
  struct SUB {
    static constexpr const char* FILTERED_ODOM = "/odometry/filtered";
    static constexpr const char* GPS_DATA = "/gps_data";
  };
  struct PUB {
    static constexpr const char* GPS_FILTERED = "/gps/filtered";
  };
};

} // namespace ros_endpoints

#endif // STATE_ESTIMATION_ROS_ENDPOINTS_HPP
