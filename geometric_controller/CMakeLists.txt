cmake_minimum_required(VERSION 3.8)
project(geometric_controller)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(Eigen3 REQUIRED)

set(Dependencies
  rclcpp
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  controller_msgs
  trajectory_msgs
  px4_msgs
  px4_ros_com
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/geometric_controller.cpp 
  src/nonlinear_attitude_control.cpp
  src/nonlinear_geometric_control.cpp
  src/jerk_tracking_control.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${Dependencies})

############
# BINARIES #
############
add_executable(geometric_controller_node
  src/geometric_controller_node.cpp
)
ament_target_dependencies(geometric_controller_node ${Dependencies})
target_link_libraries(geometric_controller_node ${PROJECT_NAME})
##########
# EXPORT #
##########
install(TARGETS ${PROJECT_NAME} geometric_controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()