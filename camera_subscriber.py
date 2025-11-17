#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

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
       
        # FPS 계산용 변수
        self.prev_time = time.time()
        self.frame_count = 0
        self.fps = 0.0

        self.get_logger().info('Camera Subscriber 노드가 시작되었습니다.')
        self.get_logger().info('Topic: /camera/image_raw')
        self.get_logger().info('Press "q" in the image window to quit')

    def listener_callback(self, msg):
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
           
            # === FPS 계산 ===
            self.frame_count += 1
            current_time = time.time()
            elapsed = current_time - self.prev_time

            if elapsed >= 1.0:  # 1초마다 FPS 갱신
                self.fps = self.frame_count / elapsed
                self.get_logger().info(f'실제 수신 FPS: {self.fps:.2f}')
                self.frame_count = 0
                self.prev_time = current_time

            # FPS를 화면에 표시
            fps_text = f'FPS: {self.fps:.2f}'
            cv2.putText(cv_image, fps_text, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # 이미지 표시
            cv2.imshow(self.window_name, cv_image)
           
            # 키 입력 대기 (1ms) - 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('종료 요청을 받았습니다.')
                rclpy.shutdown()
               
        except Exception as e:
            self.get_logger().error(f'이미지 변환 중 오류 발생: {str(e)}')

    def destroy_node(self):
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
