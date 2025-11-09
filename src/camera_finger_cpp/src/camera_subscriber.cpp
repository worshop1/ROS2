#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class CameraSubscriber : public rclcpp::Node
{
public:
    CameraSubscriber()
    : Node("camera_subscriber")
    {
        // Subscriber 생성
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw",
            10,
            std::bind(&CameraSubscriber::listener_callback, this, std::placeholders::_1));
        
        // OpenCV 윈도우 이름
        window_name_ = "Camera Feed";
        // 윈도우 생성 (자동 크기)
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::startWindowThread();
        
        RCLCPP_INFO(this->get_logger(), "Camera Subscriber 노드가 시작되었습니다.");
        RCLCPP_INFO(this->get_logger(), "Topic: /camera/image_raw");
        RCLCPP_INFO(this->get_logger(), "Press 'q' in the image window to quit");
    }
    
    ~CameraSubscriber()
    {
        // OpenCV 윈도우 닫기
        cv::destroyAllWindows();
    }

private:
    void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            // 첫 번째 메시지 수신 로그
            if (!first_msg_received_) {
                RCLCPP_INFO(this->get_logger(), "첫 번째 이미지 메시지를 받았습니다!");
                RCLCPP_INFO(this->get_logger(), "이미지 크기: %dx%d, 인코딩: %s", 
                    msg->width, msg->height, msg->encoding.c_str());
                first_msg_received_ = true;
            }
            
            // ROS Image 메시지를 OpenCV 이미지로 변환
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            
            if (cv_ptr->image.empty()) {
                RCLCPP_WARN(this->get_logger(), "변환된 이미지가 비어있습니다!");
                return;
            }
            
            // 이미지 표시 (1280x720)
            cv::imshow(window_name_, cv_ptr->image);
            
            // 키 입력 대기 (1ms) - 'q' 키를 누르면 종료
            if ((cv::waitKey(1) & 0xFF) == 'q') {
                RCLCPP_INFO(this->get_logger(), "종료 요청을 받았습니다.");
                rclcpp::shutdown();
            }
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "이미지 처리 중 예외 발생: %s", e.what());
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    std::string window_name_;
    bool first_msg_received_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<CameraSubscriber>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "예외 발생: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
