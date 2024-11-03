cmake_minimum_required(VERSION 3.8)
project(control_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(control_cmd_node src/control_cmd_node.cpp)
ament_target_dependencies(control_cmd_node
  rclcpp
  autoware_control_msgs
)

add_executable(imu_orientation src/orientation_visualizer.cpp)
ament_target_dependencies(imu_orientation
  rclcpp
  sensor_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  control_cmd_node
  imu_orientation
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME})

ament_package()
