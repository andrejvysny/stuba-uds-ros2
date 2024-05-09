#include "rclcpp/rclcpp.hpp"
#include "MotionController.cpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
using std::placeholders::_1;

#include <string>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "MotionActions.hpp"

static const int MODE_OFF = 0;
static const int MODE_MANUAL = 1;
static const int MODE_AUTO = 2;

static const std::string MANUAL_CONTROLLER_TOPIC = std::string("/controller_manual");
static const std::string AUTO_CONTROLLER_TOPIC = std::string("/controller_auto");

class KobukiController : public rclcpp::Node
{

public:
    KobukiController() : Node("kobuki_controller"), count_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Started kobuki_controller");

        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        twist_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&KobukiController::publisher_callback, this));
        RCLCPP_INFO(this->get_logger(), "Create timer and Publisher for /cmd_vel topic");

        manual_controller_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            MANUAL_CONTROLLER_TOPIC,
            10,
            std::bind(&KobukiController::manual_controller_subscriber, this, _1));
        RCLCPP_INFO(this->get_logger(), "Create subscriber for /controller_manual topic");

        auto_controller_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            AUTO_CONTROLLER_TOPIC,
            10,
            std::bind(&KobukiController::auto_controller_subscriber, this, _1));
        RCLCPP_INFO(this->get_logger(), "Create subscriber for /controller_auto topic");


        mode_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/mode",
            10,
            std::bind(&KobukiController::modeCallback, this, _1));
    }

private:
    size_t count_;
    rclcpp::TimerBase::SharedPtr twist_timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    MotionController motionController;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_subscription_;
    int currentMode = MODE_OFF;

    // ################################################################################################
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr manual_controller_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr auto_controller_subscription_;

    void manual_controller_subscriber(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (MODE_MANUAL == currentMode)
        {
            applyManualMotionAction(msg->data);
        }
    }

    void auto_controller_subscriber(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (MODE_AUTO == currentMode)
        {
            motionController.setLinearSpeed(msg->data[0]);
            motionController.setAngularSpeed(msg->data[1]);
        }
    }
    // ################################################################################################



    void publisher_callback()
    {
        twist_publisher_->publish(motionController.getTwist());
        RCLCPP_INFO(this->get_logger(), "Published data: %s", motionController.getTwistToJson().c_str());
    }

    void modeCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        currentMode = msg->data;
        

        switch (currentMode)
        {
        case MODE_OFF:
            motionController.stop();
            break;
        case MODE_MANUAL:
            RCLCPP_INFO(this->get_logger(), "Mode changed to: MANUAL");
            break;
        case MODE_AUTO:
            RCLCPP_INFO(this->get_logger(), "Mode changed to: AUTO");
            break;
        }
    }

    void applyManualMotionAction(int action)
    {
        switch (action)
        {
        case MotionActions::ACTION_INCREMENT_LINEAR_SPEED:
            motionController.increseLinearSpeed();
            break;
        case MotionActions::ACTION_DECREMENT_LINEAR_SPEED:
            motionController.decreaseLinearSpeed();
            break;
        case MotionActions::ACTION_INCREMENT_ANGULAR_SPEED:
            motionController.increaseAngularSpeed();
            break;
        case MotionActions::ACTION_DECREMENT_ANGULAR_SPEED:
            motionController.decreaseAngularSpeed();
            break;
        case MotionActions::ACTION_STOP:
            motionController.stop();
            break;
        default:
            // Handle unknown action
            break;
        }
    }
};
