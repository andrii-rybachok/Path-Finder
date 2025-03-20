#include <chrono>
#include <memory>
#include <string>

#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/videoio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
 * function to shorten the callback syntax, at the expense of making the
 * code somewhat more difficult to understand at first glance. */

class WebcamReader : public rclcpp::Node
{
public:
  WebcamReader()
      : Node("webcam_reader")
  {
    this->cap_ = cv::VideoCapture(0);

    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("webcam_reader", options);
    image_transport::ImageTransport it(node);
    imagePublisher_ = it.advertise("image_rect", 1);
    imageInfoPublisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);

    auto timer_callback =
        [this]() -> void
    {
      // auto message = std_msgs::msg::String();
      // message.data = "Hello, world! " + std::to_string(this->count_++);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      // this->publisher_->publish(message);

      this->cap_ >> this->frame_;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", this->frame_.size.dims());

      if (!this->frame_.empty())
      {
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", this->frame_).toImageMsg();
        this->imagePublisher_.publish(msg);

        sensor_msgs::msg::CameraInfo cameraInfo;
        cameraInfo.width = 1280;
        cameraInfo.height = 720;
        cameraInfo.distortion_model = "rectilinear";
        cameraInfo.d = {-0.045492745471321955,
                        0.37382800284035456,
                        0.0006460940496060854,
                        0.001973216879553601,
                        -1.0704642073825659};
        cameraInfo.k = {
            1432.032296682536,
            0,
            634.6095314832921,
            0,
            1436.7630246913845,
            361.1065883721155,
            0,
            0,
            1};

        cameraInfo.r = {
            1,
            0,
            0,
            0,
            1,
            0,
            0,
            0,
            1};

        cameraInfo.p = {
            1432.032296682536,
            0,
            634.6095314832921,
            0,
            0,
            1436.7630246913845,
            361.1065883721155,
            0,
            0,
            0,
            1,
            0};

        // cameraInfo->binning_x = 0;
        // cameraInfo->binning_y = 0;

        this->imageInfoPublisher_->publish(cameraInfo);
      }
    };
    timer_ = this->create_wall_timer(200ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::Publisher imagePublisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr imageInfoPublisher_;
  cv::Mat frame_;
  cv::VideoCapture cap_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebcamReader>());
  rclcpp::shutdown();
  return 0;
}