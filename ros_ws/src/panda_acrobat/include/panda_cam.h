
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "opencv2/opencv.hpp"


class PandaCameraNode : public rclcpp::Node
{
public:
  PandaCameraNode();

private:
  void loop();

  cv::Point trackBall(std::shared_ptr<cv::VideoCapture> cap);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
  std::thread thread_;
  std::shared_ptr<cv::VideoCapture> capture_;
};

