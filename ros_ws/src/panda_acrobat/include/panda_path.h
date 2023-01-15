
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class PandaPathNode : public rclcpp::Node
{
public:
  PandaPathNode();

private:
  void timerCallback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  int x_;
  int y_;
};

