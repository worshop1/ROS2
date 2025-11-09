#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher()
    : Node("camera_publisher")
    {
        // Publisher 생성
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
        
        // OpenCV 비디오 캡처 객체 생성 (0 = 기본 카메라) - V4L2 백엔드 사용
        cap_.open(0, cv::CAP_V4L2);
        
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "카메라를 열 수 없습니다!");
            throw std::runtime_error("카메라 초기화 실패");
        }
        
        // MJPG 포맷 설정 (해상도 및 프레임률 안정화에 도움됨)
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        
        // 해상도 설정: 640x480 (작동하던 설정을 반영)
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        
        // 프레임률 설정 (30 FPS)
        cap_.set(cv::CAP_PROP_FPS, 30);
        
        // 30 FPS로 타이머 설정 (약 33ms)
        timer_ = this->create_wall_timer(
            33ms, std::bind(&CameraPublisher::timer_callback, this));

        // 디버그 윈도우 생성
        window_name_ = "Camera Debug";
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        cv::startWindowThread();
        
        RCLCPP_INFO(this->get_logger(), "Camera Publisher 노드가 시작되었습니다.");
        RCLCPP_INFO(this->get_logger(), "Topic: /camera/image_raw");
        RCLCPP_INFO(this->get_logger(), "Resolution: 640x480, Publishing at 30 FPS");
    }
    
    ~CameraPublisher()
    {
        // 카메라 리소스 해제
        if (cap_.isOpened()) {
            cap_.release();
        }
        // 윈도우 정리
        cv::destroyAllWindows();
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        
        // 카메라에서 프레임 읽기
        cap_ >> frame;
        
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "프레임을 읽을 수 없습니다.");
            return;
        }

        // 손가락 끝점 추정: HSV 기반 피부색 마스크 -> 컨투어 -> convexHull
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        // 기본 피부색 범위 (조명에 따라 조정 필요)
        cv::inRange(hsv, cv::Scalar(0, 30, 60), cv::Scalar(20, 150, 255), mask);
        cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> finger_tips;
        if (!contours.empty()) {
            // 가장 큰 컨투어 선택
            size_t largest_idx = 0;
            double largest_area = 0;
            for (size_t i=0;i<contours.size();++i) {
                double a = cv::contourArea(contours[i]);
                if (a > largest_area) { largest_area = a; largest_idx = i; }
            }

            // 윤곽과 허프 그리기
            cv::drawContours(frame, contours, (int)largest_idx, cv::Scalar(255,0,0), 2);

            std::vector<cv::Point> hull;
            cv::convexHull(contours[largest_idx], hull);
            // 허프 그리기
            for (size_t i=0;i<hull.size();++i) {
                cv::circle(frame, hull[i], 3, cv::Scalar(0,255,0), -1);
            }

            // 간단한 heuristic: y값이 위쪽(프레임에서 작음)인 hull 포인트를 fingertip 후보로 선택
            for (size_t i=0;i<hull.size();++i) {
                cv::Point p = hull[i];
                if (p.y < frame.rows * 0.6) {
                    finger_tips.push_back(p);
                }
            }
            if (finger_tips.size() > 5) finger_tips.resize(5);

            // 표시
            for (size_t i=0;i<finger_tips.size();++i) {
                cv::circle(frame, finger_tips[i], 8, cv::Scalar(0,0,255), 2);
                cv::putText(frame, std::to_string((int)i), finger_tips[i], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255,255,255), 2);
            }
        }

        // 디버그 창에 마스크와 결과 표시 (오른쪽에 마스크를 붙여 보여줌)
        cv::Mat mask_color;
        cv::cvtColor(mask, mask_color, cv::COLOR_GRAY2BGR);
        cv::Mat disp;
        cv::hconcat(frame, mask_color, disp);
        cv::imshow(window_name_, disp);

        // OpenCV 이미지 -> ROS Image 메시지로 변환 및 발행 (annotated frame)
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";
        cv_bridge::CvImage cv_image(header, "bgr8", frame);
        sensor_msgs::msg::Image::SharedPtr msg = cv_image.toImageMsg();
        publisher_->publish(*msg);

        // 키 입력 처리: 'q'로 종료
        if ((cv::waitKey(1) & 0xFF) == 'q') {
            RCLCPP_INFO(this->get_logger(), "종료 요청(키 입력)");
            rclcpp::shutdown();
        }
    }
    
    std::string window_name_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::VideoCapture cap_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<CameraPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "예외 발생: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
