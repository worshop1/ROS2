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
