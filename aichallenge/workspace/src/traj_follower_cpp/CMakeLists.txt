cmake_minimum_required(VERSION 3.8)
project(traj_follower_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)

add_executable(traj_follower_node src/traj_follower_node.cpp)
ament_target_dependencies(traj_follower_node
  rclcpp geometry_msgs nav_msgs
  autoware_auto_planning_msgs autoware_auto_control_msgs
)

install(TARGETS traj_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
