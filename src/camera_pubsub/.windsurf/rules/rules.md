---
trigger: always_on
---

Publisher 노드 생성용 프롬프트
Write a ROS2 Humble Python node that publishes camera frames using rclpy and cv2. 
- Package name: camera_pub
- Node name: camera_publisher
- Topic name: /camera/image_raw
- Message type: sensor_msgs.msg.Image
- Use cv2.VideoCapture(0)
- Convert OpenCV frame to ROS Image using cv_bridge
- Publish at 30 FPS
- Include full code with __main__ section
- Do not omit import statements


Subscriber 노드 생성용 프롬프트
Write a ROS2 Humble Python node that subscribes to /camera/image_raw and displays the image using cv2.imshow.
- Package name: caamera_sub
- Node name: camera_subscriber
- Topic name: /camera/image_raw
- Message type: sensor_msgs.msg.Image
- Convert ROS Image message to OpenCV image using cv_bridge
- Display in real-time at the received speed
- Make sure the window closes correctly on exit
- Include full code with __main__ section
- Do not omit import statements


setup.py + package.xml 자동 생성용 프롬프트
Generate setup.py and package.xml files for a ROS2 Humble Python package.
- Package name: camera_stream
- Contains two executables: camera_publisher and camera_subscriber
- Require dependencies: rclpy, sensor_msgs, cv_bridge, std_msgs, OpenCV
- Format the files correctly for colcon build