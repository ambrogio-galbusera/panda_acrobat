
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "../include/panda_path.h"

using namespace std::chrono_literals;

PandaPathNode::PandaPathNode()
  : Node("minimal_publisher"), x_(1920/2), y_(1080/2)
{
    RCLCPP_INFO(this->get_logger(), "Constructor");
    publisher_ = this->create_publisher<std_msgs::msg::String>("panda_path", 10);

    RCLCPP_INFO(this->get_logger(), "Initializing timer");
    timer_ = this->create_wall_timer(
      500ms, std::bind(&PandaPathNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Constructor completed");
}

void PandaPathNode::timerCallback()
{
    auto message = std_msgs::msg::String();
    message.data = std::to_string(x_) + ";" + std::to_string(y_);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PandaPathNode>());
    rclcpp::shutdown();
    return 0;
}