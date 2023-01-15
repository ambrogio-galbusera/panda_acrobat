
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "opencv2/opencv.hpp"

#include "../include/panda_cam.h"

#define MINTRACKAREA 50

using namespace cv;
using namespace std::chrono_literals;

PandaCameraNode::PandaCameraNode()
  : Node("minimal_publisher"), count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Constructor");
    publisher_ = this->create_publisher<std_msgs::msg::String>("panda_pos", 10);
    RCLCPP_INFO(this->get_logger(), "Initializing timer");

    namedWindow("Tennis Ball", WINDOW_NORMAL);
    resizeWindow("Tennis Ball", 960, 540);

    namedWindow("Thresholded Tennis Ball", WINDOW_NORMAL);
    resizeWindow("Thresholded Tennis Ball", 960, 540);

    // Run loop on it's own thread
    thread_ = std::thread(std::bind(&PandaCameraNode::loop, this));
    RCLCPP_INFO(this->get_logger(), "Constructor completed");
  }

void PandaCameraNode::loop()
{
  RCLCPP_INFO(this->get_logger(), "Start thread");
  capture_ = std::make_shared<cv::VideoCapture>("/dev/video0");

  if (!capture_->isOpened()) {
    RCLCPP_ERROR(get_logger(), "cannot open device %d", 0);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Start thread [1]");
  capture_->set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
  RCLCPP_INFO(this->get_logger(), "Start thread [2]");
  capture_->set(cv::CAP_PROP_FRAME_WIDTH, 1920);

  double width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
  double height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
  double fps = capture_->get(cv::CAP_PROP_FPS);
  RCLCPP_INFO(get_logger(), "device %d open, width %g, height %g, device fps %g",
                0, width, height, fps);

  RCLCPP_INFO(this->get_logger(), "Start thread [3]");

  while (rclcpp::ok()) {
    Point p = trackBall(capture_);

    if (p.x >= 0 && p.y >= 0)
    {
      auto message = std_msgs::msg::String();
      message.data = std::to_string(p.x) + ";" + std::to_string(p.y);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
  }
}

Point PandaCameraNode::trackBall(std::shared_ptr<cv::VideoCapture> cap)
{
  Mat frame;

  //Resize large images to reduce processing load
  cap->read(frame);

  //Convert RGB to HSV colormap
  //and apply Gaussain blur
  Mat hsvFrame;
  cvtColor(frame, hsvFrame, COLOR_RGB2HSV);

  blur(hsvFrame, hsvFrame, cv::Size(1, 1));

  //Threshold 
  Scalar lowerBound = cv::Scalar(55, 100, 50);
  Scalar upperBound = cv::Scalar(90, 255, 255);
  Mat threshFrame;
  inRange(hsvFrame, lowerBound, upperBound, threshFrame);

  //Calculate X,Y centroid
  Moments m = moments(threshFrame, false);
  Point com(m.m10 / m.m00, m.m01 / m.m00);

  //Draw crosshair
  Scalar color = cv::Scalar(0, 0, 255);
  drawMarker(frame, com, color, cv::MARKER_CROSS, 50, 5);

  imshow("Tennis Ball", frame);
  imshow("Thresholded Tennis Ball", threshFrame);

  waitKey(5);

  return com;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;

  auto node = std::make_shared<PandaCameraNode>();
  executor.add_node(node);
  executor.spin();

  //rclcpp::spin(std::make_shared<PandaCameraNode>());
  rclcpp::shutdown();
  return 0;
}