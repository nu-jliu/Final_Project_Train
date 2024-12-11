#include <chrono>
#include <memory>

#include "track_detection/camera_node.hpp"

namespace track_detection
{
using namespace std::chrono_literals;

CameraNode::CameraNode()
: rclcpp::Node("camera_node")
{

  video_capture_ = std::make_shared<cv::VideoCapture>("/dev/video0");

  if (!video_capture_->isOpened()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to open video capture");
    throw std::runtime_error("Video capture failed to open");
  }

  it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
  pub_image_it_ = it_->advertise("camera/image", 10);

  timer_ = create_wall_timer(10ms, std::bind(&CameraNode::timer_callback_, this));
}

void CameraNode::timer_callback_()
{
  cv::Mat frame;
  if (!video_capture_->read(frame)) {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to read from gstreamer");
    return;
  }

  sensor_msgs::msg::Image::SharedPtr msg;
  msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  pub_image_->publish(*msg);

  pub_image_it_.publish(msg);
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<track_detection::CameraNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
