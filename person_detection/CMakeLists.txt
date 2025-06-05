cmake_minimum_required(VERSION 3.8)
project(person_detection)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(pcl_conversions REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# add executables
add_executable(laser_filter src/laser_filter.cpp)
ament_target_dependencies(laser_filter rclcpp sensor_msgs)
install(TARGETS laser_filter DESTINATION lib/${PROJECT_NAME})

add_executable(detector src/detector.cpp include/dbscan/dbscan/dbscan.cpp)
ament_target_dependencies(detector rclcpp sensor_msgs)
target_include_directories(detector PRIVATE /usr/include/eigen3 ${CMAKE_CURRENT_SOURCE_DIR}/include/dbscan)
install(TARGETS detector DESTINATION lib/${PROJECT_NAME})

add_executable(cloud_filter src/cloud_filter.cpp)
ament_target_dependencies(cloud_filter rclcpp sensor_msgs pcl_conversions)
install(TARGETS cloud_filter DESTINATION lib/${PROJECT_NAME})
target_link_libraries(cloud_filter ${PCL_LIBRARIES})

add_executable(cloud_detector src/cloud_detector.cpp include/dbscan/dbscan/dbscan.cpp)
ament_target_dependencies(cloud_detector rclcpp sensor_msgs)
target_include_directories(cloud_detector PRIVATE /usr/include/eigen3 ${CMAKE_CURRENT_SOURCE_DIR}/include/dbscan)
install(TARGETS cloud_detector DESTINATION lib/${PROJECT_NAME})

add_executable(face_closest src/face_closest.cpp)
ament_target_dependencies(face_closest rclcpp sensor_msgs geometry_msgs)
install(TARGETS face_closest DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()