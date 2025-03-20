#include <wiringPi.h>
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <json/json.h>

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>
#include <memory>
#include <queue>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "max_ros/srv/path_request.hpp"

#define STBY 17

#define AIN1 27
#define AIN2 22
#define APWM 12

#define BIN1 23
#define BIN2 24
#define BPWM 13

#define AMOTORMODIFIER -1
#define BMOTORMODIFIER 1

#define PWMRANGE 1024

#define RAD_TO_DEG 57.2958

// TODO experimental values (need to find)
#define MS_PER_METER 3900 * 0.00025 // 1st value should be accurate; 2nd value is scale conversion (scale meters per real life meters)
#define MS_PER_ROTATION 2260.0

// how often to update the direction travelling
#define MS_UPDATE_TIME 10

/*
#define STOP_CODE = 0;
#define DRIVESTRAIGHT_CODE = 1;
#define DRIVENONLINEAR_CODE = 2;
#define TURN_CODE = 3;
*/

// START TEMP CODE
// void wiringPiSetupGpio() {}

// void pinMode(int a, int b) {}

// void pwmSetRange(int a) {}

// void pwmWrite(int a, int b) {}

// void delay(int a)
// {
//     std::this_thread::sleep_for(std::chrono::milliseconds(a));
// }

// void digitalWrite(int a, int b) {}

// #define HIGH 1
// #define LOW 0
// #define OUTPUT 1
// #define PWM_OUTPUT 1

// END TEMP CODE

#define TIMER_MS 50

using namespace curlpp::Options;

class Controls : public rclcpp::Node
{
public:
    // TODO these are temp for testing - delete later
    // maybe not the setup one though
    void TEST_setup() { setup(); }
    void TEST_requestPath(std::array<unsigned char, 4> destinationName)
    {
        requestPath(destinationName);
    }

    Controls()
        : Node("controls")
    {
        RCLCPP_INFO(this->get_logger(), "hey");
        auto object_detection_callback =
            [this](std_msgs::msg::String::UniquePtr msg) -> void
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

            if (msg->data == "Object detected")
            {
                this->interrupt();
            }

            if (msg->data == "Object gone")
            {
                this->resume();
            }
        };

        /*auto pathfinding_callback =
            [this](std_msgs::msg::Float64MultiArray::UniquePtr msg) -> void
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%f', '%f'", msg->data.multiarray(0), msg->data.multiarray(1));
            directions.push({msg->data.multiarray(0),msg->data.multiarray(1)});
        };*/

        auto request_timer_callback =
            [this]() -> void
        {
            // Don't look for new instructions if not done with current destination
            if (!directions.empty())
            {
                return;
            }

            std::ostringstream appResponse;
            curlpp::Easy appRequest;

            appRequest.setOpt<curlpp::options::Url>("http://192.168.64.1:5000/status");
            appRequest.setOpt<curlpp::options::HttpGet>(true);
            appRequest.setOpt<curlpp::options::WriteStream>(&appResponse);

            appRequest.perform();

            Json::CharReaderBuilder reader;
            Json::Value jsonData;
            std::string errs;

            std::istringstream responseIStream(appResponse.str());

            if (!Json::parseFromStream(reader, responseIStream, &jsonData, &errs))
            {
                return;
            }

            if (jsonData["destination"].isNull())
            {
                return;
            }

            std::string destination = jsonData["destination"].asString();

            std::string building = destination.substr(0, destination.find(' '));
            RCLCPP_INFO(this->get_logger(), "Going to building %s", building.c_str());
            std::array<unsigned char, 4> locationCode;

            std::copy(building.begin(), building.end(), locationCode.begin());

            requestPath(locationCode);
        };

        auto timer_callback =
            [this]() -> void
        {
            auto message = std_msgs::msg::String();
            message.data = "I am at: " + std::to_string(curX) + ", " + std::to_string(curY) + ", " + std::to_string(curTheta);

            // TODO: re-add this when we have a way to convert from lat/lon to x, y
            // double latitude = jsonData["location"]["latitude"].asDouble();
            // double longitude = jsonData["location"]["longitude"].asDouble();

            // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            this->publisher_->publish(message);

            update();
        };

        auto localization_callback =
            [this](std_msgs::msg::Float64MultiArray::UniquePtr msg) -> void
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%f', '%f', '%f", msg->data[0], msg->data[1], msg->data[2]);
            curX = msg->data[0];
            curY = msg->data[1];
            curTheta = msg->data[2];
        };

        publisher_ = this->create_publisher<std_msgs::msg::String>("controls_pub", 10);
        // subscriber1_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("pathfinding_sub", 10, pathfinding_callback);
        subscriber2_ = this->create_subscription<std_msgs::msg::String>("object_detection", 10, object_detection_callback);
        subscriber3_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("localization_sub", 10, localization_callback);
        client_ = this->create_client<max_ros::srv::PathRequest>("path_planning");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(TIMER_MS), timer_callback);
        requestTimer_ = this->create_wall_timer(std::chrono::seconds(5), request_timer_callback);

        TEST_setup();
        delay(5000);
    }

    void shutdown()
    {
        stop();
        enableMotorControl(0);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr requestTimer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber1_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber2_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber3_;
    rclcpp::Client<max_ros::srv::PathRequest>::SharedPtr client_;
    std::queue<std::vector<int>> directions;

    double timeElapsed = 0;

    // for checking interruption
    bool interrupted = false;
    double curX = 0.0, curY = 0.0, curTheta = 0.0;

    // 1 for enable, 0 for disable
    void enableMotorControl(int enable)
    {
        digitalWrite(STBY, enable ? HIGH : LOW);
    }

    // Sets up the GPIOs for the motor control
    // and disables the motor control by default
    void setup()
    {
        // uses BCM numbering of the GPIOs and directly accesses the GPIO registers.
        wiringPiSetupGpio();
        // if (wiringPiSetupGpio() == -1)
        //     exit(1);

        // pin mode ..(INPUT, OUTPUT, PWM_OUTPUT, GPIO_CLOCK)
        pinMode(STBY, OUTPUT);

        pinMode(AIN1, OUTPUT);
        pinMode(AIN2, OUTPUT);
        pinMode(APWM, PWM_OUTPUT);

        pinMode(BIN1, OUTPUT);
        pinMode(BIN2, OUTPUT);
        pinMode(BPWM, PWM_OUTPUT);

        pwmSetRange(PWMRANGE);

        enableMotorControl(1);
    }

    void stop()
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, HIGH);
        pwmWrite(APWM, 0);

        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, HIGH);
        pwmWrite(BPWM, 0);
    }

    // Motor: 0 for left and 1 for right
    // Speed: between -1 and 1
    void setMotorSpeed(int motor, double speed)
    {
        if (speed == 0.0)
        {
            stop();
            return;
        }

        digitalWrite(motor ? BIN1 : AIN1, (motor ? BMOTORMODIFIER : AMOTORMODIFIER) * speed > 0.0);
        digitalWrite(motor ? BIN2 : AIN2, (motor ? BMOTORMODIFIER : AMOTORMODIFIER) * speed < 0.0);

        int pwmSpeed = speed > 0.0 ? static_cast<int>(speed * (PWMRANGE - 1)) : static_cast<int>(-speed * (PWMRANGE - 1));
        pwmWrite(motor ? BPWM : APWM, pwmSpeed);
    }

    void update()
    {
        if (!directions.empty() && atTargetLocation())
        {
            directions.pop();
        }

        if (directions.empty() || interrupted)
        {
            return;
        }

        double angleDelta = std::fmod(((atan2(directions.front()[0] - curX, directions.front()[1] - curY) * 180 / M_PI - curTheta) + 540), 360) - 180;
        // RCLCPP_INFO(this->get_logger(), "%f, %f, %f", dX, dY, angleDelta);
        if (abs(angleDelta) > 10)
        {
            setMotorSpeed(0, angleDelta > 0 ? 0.5 : -0.5);
            setMotorSpeed(1, angleDelta > 0 ? -0.5 : 0.5);

            curTheta = fmod(angleDelta > 0 ? (curTheta + (1 / MS_PER_ROTATION) * TIMER_MS * 360) : (curTheta - (1 / MS_PER_ROTATION) * TIMER_MS * 360), 360);
        }
        else
        {
            setMotorSpeed(0, 0.5);
            setMotorSpeed(1, 0.5);

            curX += (1 / MS_PER_METER) * TIMER_MS * sin(M_PI * curTheta / 180);
            curY += (1 / MS_PER_METER) * TIMER_MS * cos(M_PI * curTheta / 180);
        }
    }

    bool atTargetLocation()
    {
        double dX = directions.front()[0] - curX;
        double dY = directions.front()[1] - curY;
        return sqrt(dX * dX + dY * dY) <= 10;
    }

    void interrupt()
    {
        interrupted = true;
        stop();
    }

    void resume()
    {
        interrupted = false;
    }

    void requestPath(std::array<unsigned char, 4> destinationName)
    {
        using namespace std::chrono_literals;

        auto request = std::make_shared<max_ros::srv::PathRequest::Request>();
        request->currentx = curX;
        request->currenty = curY;
        std::array<unsigned char, 4> goal_array;
        goal_array = destinationName;
        request->goal = goal_array;
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                // return 0;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto callback = [this](rclcpp::Client<max_ros::srv::PathRequest>::SharedFuture result) -> void
        {
            auto r = result.get();
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Path length: %ld", r->pathx.size());
            for (unsigned long int i = 0; i < r->pathx.size(); ++i)
            {
                directions.push({r->pathx[i], r->pathy[i]});
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pushed to queue: %ld, %ld", r->pathx[i], r->pathy[i]);
            }
        };
        auto result = client_->async_send_request(request, callback);
    }
};

int main(int argc, char *argv[])
{
    printf("running main\n");

    rclcpp::init(argc, argv);
    std::shared_ptr<Controls> node = std::make_shared<Controls>();

    // node->TEST_requestPath({'S', 'T', 'C', '\0'});
    // node->TEST_setup();
    rclcpp::spin(node);
    printf("setup done\n");
    node->shutdown();
    printf("testing - going to STC\n");
    // node->TEST_requestPath({'S', 'T', 'C', '\0'});

    /*
        driveStraight(0.5, 1);
        delay(1000);
        std::cerr << "drove\n";
        turn(-90);
        delay(1000);
        driveStraight(0.4, 0.5);

        while(1){
            timeElapsed += (double)MS_UPDATE_TIME / (double)1000;
            driveNonLinear(sin(timeElapsed),cos(timeElapsed));
            delay(MS_UPDATE_TIME);
        }*/

    // rclcpp::shutdown();

    return 0;
}
