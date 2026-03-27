cmake_minimum_required(VERSION 3.8)
project(flap_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Dipendenze
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sail_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

set(CONTROL_DEPS rclcpp std_msgs sail_msgs)

# 2. Include e Installazione Risorse
include_directories(include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# 3. Executables (Solo quelli che ci servono davvero!)

# -- Il Cervello --
add_executable(pid_controller_node 
  src/pid_controller_node.cpp
  src/ConfigManager.cpp
  src/BoatStateMachine.cpp
  src/PIDMath.cpp
  src/GuiManager.cpp
)
ament_target_dependencies(pid_controller_node ${CONTROL_DEPS} std_srvs geometry_msgs sensor_msgs tf2)

# -- Il Braccio --
add_executable(flap_controller_node src/flap_controller_node.cpp)
ament_target_dependencies(flap_controller_node ${CONTROL_DEPS})

# 4. Installazione Target (Rimosso il vecchiume)
install(TARGETS
  pid_controller_node flap_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()