cmake_minimum_required(VERSION 3.8)
project(rfanslidar_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
 
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

link_directories(/usr/local/lib)
include_directories(
  include
  lib/include
  lib/src
)
add_library(${PROJECT_NAME} SHARED
  src/driver.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
)
rclcpp_components_register_nodes(${PROJECT_NAME} "MiYALAB::ROS2::RPLiDAR")

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

# yaml
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME}/)

# launch
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

ament_package()