cmake_minimum_required(VERSION 3.8)
project(ros2_teleop)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Add the executable (target name must match here)
add_executable(ros2_teleop_node src/ros2_teleop.cpp)

# Link dependencies to the target
ament_target_dependencies(ros2_teleop_node
  rclcpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
)

# Install the executable
install(TARGETS
  ros2_teleop_node
  DESTINATION lib/ros2_teleop
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/ros2_teleop
)
ament_package()