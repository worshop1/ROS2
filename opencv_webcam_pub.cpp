#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("opencv_webcam_pub");

    // CompressedImage 전용 ROS2 퍼블리셔 설정
    auto publisher = node->create_publisher<sensor_msgs::msg::CompressedImage>("/image_raw/compressed", 10);

    // 카메라 초기화
    VideoCapture cap(0, cv::CAP_V4L2);
    if (!cap.isOpened())
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to open the camera.");
        return 1;
    }

    // 카메라 해상도 및 FPS 설정
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_FPS, 15); // FPS 제한

    // 루프 시작
    while (rclcpp::ok())
    {
        Mat frame, flipped_frame;
        cap >> frame;
        if (frame.empty()) continue;

        flipped_frame = frame;
        // 상하 반전 (0), 좌우반전(1), 상하좌우 반전(-1)
        //flip(frame, flipped_frame, 0);

        // JPEG 압축
        std::vector<uchar> buf;
        std::vector<int> params = {IMWRITE_JPEG_QUALITY, 80};
        imencode(".jpg", flipped_frame, buf, params);

        // ROS2 메시지 생성
        auto msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
        msg->format = "jpeg";
        msg->data = std::move(buf);
        msg->header.stamp = node->now();

        // 메시지 발행
        publisher->publish(*msg);

        rclcpp::sleep_for(33ms); // 30 FPS 기준
    }

    cap.release();
    rclcpp::shutdown();
    return 0;
}
