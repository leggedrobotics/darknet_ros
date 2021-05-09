cmake_minimum_required(VERSION 3.5)
project(darknet_ros_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoundingBox.msg"
  "msg/BoundingBoxes.msg"
  "msg/ObjectCount.msg"
  "action/CheckForObjects.action"
  DEPENDENCIES builtin_interfaces action_msgs geometry_msgs sensor_msgs std_msgs 
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
