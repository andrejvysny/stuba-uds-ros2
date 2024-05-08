#include "rclcpp/rclcpp.hpp"


#include "MotionController.cpp"



int main(int argc, char * argv[])
{
    MotionController mc;


        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<KobukiControl>());
        rclcpp::shutdown();
        return 0;
}
