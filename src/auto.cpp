#include "rclcpp/rclcpp.hpp"

#include "AutoController.cpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoController>());
    rclcpp::shutdown();
    return 0;
}
