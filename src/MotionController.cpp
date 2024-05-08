#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include "geometry_msgs/msg/twist.hpp"


class MotionController {
public:
    MotionController() {
        stop();
        // Constructor implementation
    }

    ~MotionController() {
        stop();
        // Destructor implementation
    }

    geometry_msgs::msg::Twist getTwist()
    {
        twist.linear.x = linearX;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = angularZ;

        return twist;
    }


    std::string getTwistToJson() const
    {
        std::string result = "{ linear_x:" + std::to_string(twist.linear.x) + ", angular_z: " + std::to_string(twist.angular.z) + " }";
        return result;
    }

    void stop()
    {
        linearX = 0.0;
        angularZ = 0.0;
    }

    void increseLinearSpeed()
    {
        linearX += 0.1;
    }

    void decreaseLinearSpeed()
    {
        linearX -= 0.1;
    }

    void increaseAngularSpeed()
    {
        angularZ += 0.1;
    }

    void decreaseAngularSpeed()
    {
        angularZ -= 0.1;
    }

private:

    geometry_msgs::msg::Twist twist;

    double linearX = 0.0;
    double angularZ = 0.0;
};

#endif // MOTIONCONTROLLER_H