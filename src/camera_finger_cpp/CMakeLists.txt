cmake_minimum_required(VERSION 3.5)
project(camera_finger_cpp)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

add_executable(camera_publisher src/camera_publisher.cpp)
ament_target_dependencies(camera_publisher rclcpp sensor_msgs cv_bridge image_transport)
# provide build/install include paths if package provides headers
target_include_directories(camera_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# link OpenCV libraries (plain signature to avoid CMake ament conflicts)
target_link_libraries(camera_publisher ${OpenCV_LIBS})

add_executable(camera_subscriber src/camera_subscriber.cpp)
ament_target_dependencies(camera_subscriber rclcpp sensor_msgs cv_bridge image_transport)
target_include_directories(camera_subscriber
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(camera_subscriber ${OpenCV_LIBS})

install(TARGETS
  camera_publisher
  camera_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers if any exist under include/
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/
    DESTINATION include
  )
  ament_export_include_directories(include)
endif()

ament_package()
