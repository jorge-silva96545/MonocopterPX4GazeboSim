cmake_minimum_required(VERSION 3.16)
project(gz_magnetometer_realism_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(gz-cmake3 REQUIRED)

find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)

add_library(gz_magnetometer_realism_plugin SHARED
  src/MagnetometerPhysical.cc
)

target_include_directories(gz_magnetometer_realism_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(gz_magnetometer_realism_plugin PUBLIC
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-math7::gz-math7
  gz-common5::gz-common5
)

install(
  TARGETS gz_magnetometer_realism_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)
