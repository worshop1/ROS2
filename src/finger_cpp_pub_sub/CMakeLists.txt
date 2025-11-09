cmake_minimum_required(VERSION 3.8)
project(finger_cpp_pub_sub)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(finger_detector_node src/finger_detector.cpp)
ament_target_dependencies(finger_detector_node rclcpp sensor_msgs cv_bridge image_transport OpenCV)

add_executable(finger_listener_node src/finger_listener.cpp)
ament_target_dependencies(finger_listener_node rclcpp sensor_msgs cv_bridge image_transport OpenCV std_msgs)

install(TARGETS
  finger_detector_node
  finger_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
