#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class FingerDetectorNode : public rclcpp::Node
{
public:
  FingerDetectorNode()
  : Node("finger_detector")
  {
    image_sub_ = image_transport::create_subscription(this, "/camera/image_raw",
      std::bind(&FingerDetectorNode::imageCallback, this, _1), "raw");

    image_pub_ = image_transport::create_publisher(this, "/finger_detection/image_annotated");

    coords_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/finger_detection/coords", 10);

    RCLCPP_INFO(this->get_logger(), "Finger detector node started");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Received image: %dx%d", frame.cols, frame.rows);

    // Use YCrCb color space for better skin detection
    cv::Mat ycrcb;
    cv::cvtColor(frame, ycrcb, cv::COLOR_BGR2YCrCb);
    
    // Define skin color range in YCrCb
    cv::Mat mask;
    cv::inRange(ycrcb, cv::Scalar(0, 133, 77), cv::Scalar(255, 173, 127), mask);
    
    // Show the mask for debugging
    cv::Mat debug = frame.clone();
    debug.setTo(cv::Scalar(0, 255, 0), mask);  // Green overlay
    cv::addWeighted(frame, 0.7, debug, 0.3, 0.0, debug);

    // Morphological operations to clean up the mask
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
    cv::erode(mask, mask, element);
    cv::dilate(mask, mask, element, cv::Point(-1,-1), 2);  // More dilation
    
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std_msgs::msg::Float32MultiArray coords_msg;

    for (const auto & c : contours) {
      double area = cv::contourArea(c);
      if (area < 1000 || area > 50000) continue;  // Larger area range for hand/finger detection
      cv::Moments m = cv::moments(c);
      if (m.m00 == 0) continue;
      int cx = int(m.m10 / m.m00);
      int cy = int(m.m01 / m.m00);
      // Draw contour and center point
      cv::drawContours(frame, std::vector<std::vector<cv::Point>>{c}, -1, cv::Scalar(0,255,0), 2);
      cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(0,0,255), -1);
      cv::putText(frame, "Finger", cv::Point(cx-20, cy-10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 2);

      // normalized coords
      float nx = float(cx) / float(frame.cols);
      float ny = float(cy) / float(frame.rows);
      coords_msg.data.push_back(nx);
      coords_msg.data.push_back(ny);
    }

    RCLCPP_DEBUG(this->get_logger(), "Detected contours: %zu", contours.size());

    // publish annotated image
    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    image_pub_.publish(out_msg);

    // publish coords
    coords_pub_->publish(coords_msg);
  }

  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr coords_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FingerDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
