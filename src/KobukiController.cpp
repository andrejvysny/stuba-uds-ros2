#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <iostream>
#include <map>
#include <chrono>
#include <ncurses.h>
#include <cmath>
#include <random>


class KobukiController : public rclcpp::Node
{   

//https://github.com/ros2/ros2_documentation/blob/foxy/source/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.rst

public:
    KobukiController() : Node("kobuki_control")
    {
        // Initialize publishers, subscribers, and other variables here
    }

    MinimalPublisher() : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

    // Implement required methods here

private:
    // Declare private member variables here



    void KobukiController::publisher()
    {
        // Declare a publisher member variable in the class
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

        // Create the publisher in the constructor
        publisher_ = this->create_publisher<std_msgs::msg::String>("/example", 10);

        // Publish a message
        std_msgs::msg::String message;
        message.data = "Hello, world!";
        publisher_->publish(message);


        void publisher()
        {
            // Declare a publisher member variable in the class
            rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
            // Create the publisher in the constructor
            publisher_ = this->create_publisher<std_msgs::msg::String>("/example", 10);

            // Create a timer with a period of 1 second
            auto timer = this->create_wall_timer(std::chrono::seconds(1), [publisher_]() {
                // Publish a message
                std_msgs::msg::String message;
                message.data = "Hello, world!";
                publisher_->publish(message);
            });
        }
    }



};

