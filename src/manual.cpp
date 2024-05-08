#include "rclcpp/rclcpp.hpp"

#include "ManualController.cpp"


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualController>());
    rclcpp::shutdown();
    return 0;
}
