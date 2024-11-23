cmake_minimum_required(VERSION 3.8)
project(common_whill_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(whill_msgs REQUIRED)

include_directories(
  include
)

add_executable(whill_controller src/whill_controller.cpp)
ament_target_dependencies(whill_controller 
  rclcpp
  sensor_msgs
  autoware_vehicle_msgs
  tier4_vehicle_msgs
  autoware_control_msgs
)

add_executable(whill_interface src/whill_interface.cpp)
ament_target_dependencies(whill_interface
  rclcpp
  whill_msgs
  tier4_vehicle_msgs
  autoware_vehicle_msgs
  autoware_control_msgs
)

install(TARGETS
  whill_controller
  whill_interface
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
