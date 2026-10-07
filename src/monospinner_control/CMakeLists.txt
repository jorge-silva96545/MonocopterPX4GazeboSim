cmake_minimum_required(VERSION 3.16)
project(monospinner_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)

add_executable(monospinner_mode
  src/monospinner_mode.cpp
)
target_link_libraries(monospinner_mode PUBLIC
  Eigen3::Eigen
  rclcpp::rclcpp
  px4_ros2_cpp::px4_ros2_cpp
)

install(TARGETS
  monospinner_mode
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
