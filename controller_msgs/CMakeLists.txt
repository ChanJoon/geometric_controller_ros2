cmake_minimum_required(VERSION 3.8)
project(controller_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AttitudeTarget.msg"
  "msg/FlatTarget.msg"
  "msg/PositionTarget.msg"
  "msg/State.msg"
  "srv/CommandBool.srv"
  "srv/SetMode.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

include_directories(include)

ament_export_dependencies(geometry_msgs sensor_msgs std_msgs)
ament_package()