cmake_minimum_required(VERSION 3.8)
project(our_can_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# CAN 드라이버 노드
add_executable(can_driver_node
  src/can_driver.cpp
)
ament_target_dependencies(can_driver_node
  rclcpp
  geometry_msgs
  sensor_msgs
  joy
)

# 휠 오도메트리 노드
add_executable(wheel_odom_node
  src/wheel_odom.cpp
)
ament_target_dependencies(wheel_odom_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

install(TARGETS
  can_driver_node
  wheel_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()