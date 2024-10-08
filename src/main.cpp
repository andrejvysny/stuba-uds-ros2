#include "rclcpp/rclcpp.hpp"

#include "KobukiController.cpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KobukiController>());
    rclcpp::shutdown();
    return 0;
}
