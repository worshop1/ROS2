finger_cpp_pub_sub
===================

C++ ROS2 package that subscribes to a camera image, runs a simple fingertip detector (OpenCV placeholder), publishes an annotated image and finger coordinates.

Topics:
- Subscribes: /camera/image_raw (sensor_msgs/Image)
- Publishes: /finger_detection/image_annotated (sensor_msgs/Image)
- Publishes: /finger_detection/coords (std_msgs/Float32MultiArray) - normalized [x,y,...]

Build:
  colcon build --packages-select finger_cpp_pub_sub

Run:
  source install/setup.bash
  ros2 run finger_cpp_pub_sub finger_detector_node

Notes:
- This is a simple placeholder detector using thresholding and contours. For production use, integrate MediaPipe Hands or a trained model.
