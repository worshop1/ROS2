#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;

class FingerListenerNode : public rclcpp::Node
{
public:
  FingerListenerNode()
  : Node("finger_listener")
  {
    image_sub_ = image_transport::create_subscription(this, "/finger_detection/image_annotated",
      std::bind(&FingerListenerNode::imageCallback, this, _1), "raw");

    coords_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/finger_detection/coords", 10,
      std::bind(&FingerListenerNode::coordsCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Finger listener node started");
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

    // show image if possible
    if (!frame.empty()) {
      cv::imshow("finger_annotated", frame);
      cv::waitKey(1);
    }
  }

  void coordsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // print normalized coordinates
    if (msg->data.empty()) {
      RCLCPP_INFO(this->get_logger(), "No fingers detected");
      return;
    }
    std::ostringstream ss;
    ss << "Finger coords (normalized): ";
    for (size_t i = 0; i + 1 < msg->data.size(); i += 2) {
      ss << "[" << msg->data[i] << "," << msg->data[i+1] << "] ";
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }

  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr coords_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FingerListenerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
