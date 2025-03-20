#include <cstdio>
#include <wiringPi.h>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// TODO: make sure these are the right pins
#define TRIGGER 5
#define ECHO 6

class ObjectDetection : public rclcpp::Node
{
public:
    ObjectDetection()
        : Node("minimal_publisher")
    {
        wiringPiSetupGpio();

        pinMode(TRIGGER, OUTPUT);
        pinMode(ECHO, INPUT);

        isBlocked_ = false;

        publisher_ = this->create_publisher<std_msgs::msg::String>("object_detection", 10);
        auto timer_callback =
            [this]() -> void
        {
            double distance = this->getDistance();
            RCLCPP_INFO(this->get_logger(), "Distance: %f", distance);

            if (distance < 50 && !this->isBlocked_) // 0.5)
            {
                auto message = std_msgs::msg::String();
                message.data = "Object detected";
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                this->publisher_->publish(message);
                this->isBlocked_ = true;
            }
            else if (distance >= 50 && this->isBlocked_)
            {
                auto message = std_msgs::msg::String();
                message.data = "Object gone";
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                this->publisher_->publish(message);
                this->isBlocked_ = false;
            }
        };
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    bool isBlocked_ = false;

    double getDistance()
    {
        digitalWrite(TRIGGER, HIGH);
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        digitalWrite(TRIGGER, LOW);

        auto start = std::chrono::system_clock::now();
        auto end = std::chrono::system_clock::now();

        while (digitalRead(ECHO) == LOW)
        {
            start = std::chrono::system_clock::now();
        }

        while (digitalRead(ECHO) == HIGH)
        {
            end = std::chrono::system_clock::now();
        }

        double timeElapsed = (end - start).count();
        double distance = (timeElapsed * 0.00004300) / 2;

        return distance;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetection>());
    rclcpp::shutdown();

    return 0;
}
