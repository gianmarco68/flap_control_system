cmake_minimum_required(VERSION 3.8)
project(state_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dipendenze Strettamente Necessarie
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED) # <-- AGGIUNTO PER L'ODOMETRIA
find_package(tf2 REQUIRED)

# 2. Include e Installazione file (Cartelle launch e config)
include_directories(include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# 3. Executables

# --- Nodo Altezza ---
add_executable(height_estimator_node src/height_estimator_node.cpp)
ament_target_dependencies(height_estimator_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  tf2
)

# --- Nodi Catena GPS ---
add_executable(gps_header_fixer_node src/gps_header_fixer_node.cpp)
ament_target_dependencies(gps_header_fixer_node 
  rclcpp 
  sensor_msgs
)

add_executable(gps_input_adapter_node src/gps_input_adapter_node.cpp)
ament_target_dependencies(gps_input_adapter_node 
  rclcpp 
  sensor_msgs 
  nav_msgs
)

add_executable(gps_output_adapter_node src/gps_output_adapter_node.cpp)
ament_target_dependencies(gps_output_adapter_node 
  rclcpp 
  sensor_msgs 
  nav_msgs
)

# 4. Installazione Target
install(TARGETS
  height_estimator_node
  gps_header_fixer_node
  gps_input_adapter_node
  gps_output_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()