cmake_minimum_required(VERSION 3.16)
project(gz_monospinner_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# An ament package (not a plain CMake project like gz_monospinner_plugin) because the
# system subscribes to PX4's EKF2 estimate through px4_msgs, which only exists in the
# colcon workspace. Built by `colcon build` (scripts/build_all.sh ros).
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Same gz libraries, resolved the same way, as gz_monospinner_plugin: the system is loaded
# into the same gz-sim process.
find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)

# The control law on its own: no gz, no ROS -- reusable from a PX4 ROS 2 mode later.
add_library(reduced_attitude_controller STATIC
  src/ReducedAttitudeController.cpp
)
set_target_properties(reduced_attitude_controller PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(reduced_attitude_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(reduced_attitude_controller PUBLIC Eigen3::Eigen)

add_library(gz_monospinner_control SHARED
  src/MonospinnerController.cc
)
target_link_libraries(gz_monospinner_control PUBLIC
  reduced_attitude_controller
  rclcpp::rclcpp
  ${px4_msgs_TARGETS}
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
  gz-common5::gz-common5
)

install(
  TARGETS gz_monospinner_control reduced_attitude_controller
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/run_closed_loop.sh scripts/record_controller.py
  scripts/send_setpoint.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
