cmake_minimum_required(VERSION 3.5)
project(localization)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TagDetection.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

#install(DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}
#)

install(PROGRAMS
  localization/ar_detection.py
  #drone_localization/position_estimation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

