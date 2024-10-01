cmake_minimum_required(VERSION 3.5)
project(can_reader)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(can_receiver_node src/can_reader_node.cpp)

ament_target_dependencies(can_receiver_node
  rclcpp
  std_msgs
  ros2_socketcan
  nav_msgs
  tf2_ros
)

install(TARGETS
  can_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
