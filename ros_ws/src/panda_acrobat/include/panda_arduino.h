#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


class PandaArduinoNode : public rclcpp::Node
{
public:
    PandaArduinoNode();

private:
    void posCallback(const std_msgs::msg::String & msg) const;
    void pathCallback(const std_msgs::msg::String & msg) const;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posSubscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pathSubscription_;

    int serialPort_;
};

