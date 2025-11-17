#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

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
       
        # MJPG 포맷 설정 (성능 및 안정성 향상)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
       
        # 해상도 설정: 800x600
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1024)
       
        # 프레임률 설정 (목표 15 FPS)
        self.cap.set(cv2.CAP_PROP_FPS, 15)
       
        # 타이머 설정: 15 FPS 목표
        timer_period = 1.0 / 60.0  # 0.0667초
        self.timer = self.create_timer(timer_period, self.timer_callback)
       
        # CvBridge 객체 생성
        self.bridge = CvBridge()

        # === FPS 측정용 변수 초기화 ===
        self.publish_count = 0
        self.last_log_time = time.time()

        # === ROS 메시지에 타임스탬프 추가용 (옵션) ===
        self.get_clock().now()  # ROS 시간 초기화

        self.get_logger().info('Camera Publisher 노드가 시작되었습니다.')
        self.get_logger().info('Topic: /camera/image_raw')
        self.get_logger().info('Resolution: 800x600, 목표 15 FPS')

    def timer_callback(self):
        # 카메라에서 프레임 읽기
        ret, frame = self.cap.read()
       
        if ret:
            # OpenCV 이미지를 ROS Image 메시지로 변환
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            
            # === ROS 헤더에 현재 시간 타임스탬프 추가 (정확한 시간 측정용) ===
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_frame'

            # 메시지 발행
            self.publisher_.publish(msg)

            # === FPS 계산 및 1초마다 로그 출력 ===
            self.publish_count += 1
            current_time = time.time()
            elapsed = current_time - self.last_log_time

            if elapsed >= 1.0:  # 1초마다 출력
                actual_fps = self.publish_count / elapsed
                self.get_logger().info(f'실제 발행 FPS: {actual_fps:.2f} (목표: 15.0)')
                self.publish_count = 0
                self.last_log_time = current_time

        else:
            self.get_logger().warning('프레임을 읽을 수 없습니다.')

    def destroy_node(self):
        # 카메라 리소스 해제
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
   
    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        camera_publisher.get_logger().info('키보드 인터럽트로 종료됩니다.')
    except Exception as e:
        camera_publisher.get_logger().error(f'예외 발생: {str(e)}')
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
