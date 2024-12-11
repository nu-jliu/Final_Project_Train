#ifndef TRACK_DETECTION__CAMERA_NODE_HPP_
#define TRACK_DETECTION__CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>


namespace track_detection
{
class CameraNode : public rclcpp::Node
{
public:
  CameraNode();

private:
  std::shared_ptr<cv::VideoCapture> video_capture_;

  std::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::Publisher pub_image_it_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;

  void timer_callback_();
};
}  // namespace track_detection
#endif  // TRACK_DETECTION__CAMERA_NODE_HPP_
