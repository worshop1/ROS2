# ROS2 카메라 Pub/Sub 패키지 개발 튜토리얼 - Part 1

## 📚 개요

### 학습 목표
- ROS2 Humble에서 Python과 C++로 카메라 영상을 발행(Publish)하고 구독(Subscribe)하는 노드 개발
- OpenCV를 활용한 카메라 영상 처리
- `cv_bridge`를 사용한 ROS2 Image 메시지와 OpenCV 이미지 간 변환

### 개발 환경
- **OS:** Ubuntu 22.04
- **ROS2 버전:** Humble
- **카메라:** 노트북 내장 카메라 (USB 카메라)
- **해상도:** 1280x720
- **프레임률:** 15 FPS
- **토픽:** `/camera/image_raw`
- **메시지 타입:** `sensor_msgs/msg/Image`

---

## Python ROS2 패키지 개발

### 1. 패키지 생성

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python camera_pubsub \
  --dependencies rclpy sensor_msgs cv_bridge std_msgs
```

### 2. Camera Publisher 구현

파일: `camera_pubsub/camera_publisher.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Publisher 생성
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # OpenCV 비디오 캡처 객체 생성 (0 = 기본 카메라)
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error('카메라를 열 수 없습니다!')
            raise RuntimeError('카메라 초기화 실패')
        
        # MJPG 포맷 설정 (해상도 및 프레임률 안정화에 도움됨)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        # 해상도 설정: 1280x720
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        # 프레임률 설정 (15 FPS)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
        
        # 15 FPS로 타이머 설정
        timer_period = 1.0 / 15.0  # 15 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # CvBridge 객체 생성
        self.bridge = CvBridge()
        
        self.get_logger().info('Camera Publisher 노드가 시작되었습니다.')
        self.get_logger().info('Topic: /camera/image_raw')
        self.get_logger().info('Resolution: 1280x720, Publishing at 15 FPS')
    
    def timer_callback(self):
        # 카메라에서 프레임 읽기
        ret, frame = self.cap.read()
        
        if ret:
            # OpenCV 이미지를 ROS Image 메시지로 변환
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            
            # 메시지 발행
            self.publisher_.publish(msg)
        else:
            self.get_logger().warning('프레임을 읽을 수 없습니다.')
    
    def destroy_node(self):
        # 카메라 리소스 해제
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    camera_publisher = CameraPublisher()
    
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```

### 3. Camera Subscriber 구현

파일: `camera_pubsub/camera_subscriber.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Subscriber 생성
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        
        # CvBridge 객체 생성
        self.bridge = CvBridge()
        
        # OpenCV 윈도우 이름
        self.window_name = 'Camera Feed'
        
        self.get_logger().info('Camera Subscriber 노드가 시작되었습니다.')
        self.get_logger().info('Topic: /camera/image_raw')
        self.get_logger().info('Press "q" in the image window to quit')
    
    def listener_callback(self, msg):
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 이미지 표시 (1280x720)
            cv2.imshow(self.window_name, cv_image)
            
            # 키 입력 대기 (1ms) - 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('종료 요청을 받았습니다.')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'이미지 변환 중 오류 발생: {str(e)}')
    
    def destroy_node(self):
        # OpenCV 윈도우 닫기
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    camera_subscriber = CameraSubscriber()
    
    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        camera_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

```

### 4. setup.py 수정

```python
from setuptools import find_packages, setup

package_name = 'camera_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jarabot',
    maintainer_email='jarabot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = camera_pubsub.camera_publisher:main',
            'camera_subscriber = camera_pubsub.camera_subscriber:main',
        ],
    },
)

```

### 5. package.xml에 의존성 추가

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>camera_pubsub</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="jarabot@todo.todo">jarabot</maintainer>
  <license>TODO: License declaration</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

```

### 6. 빌드 및 실행

```bash
cd ~/ros2_ws
colcon build --packages-select camera_pubsub
source install/setup.bash

# 터미널 1
ros2 run camera_pubsub camera_publisher

# 터미널 2
ros2 run camera_pubsub camera_subscriber
```

---

## 다음: Part 2에서 C++ 구현을 다룹니다
