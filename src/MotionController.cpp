#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include "geometry_msgs/msg/twist.hpp"


class MotionController {
public:
    MotionController() {
        // Constructor implementation
    }

    ~MotionController() {
        // Destructor implementation
    }

    // Add other public methods and member variables as needed

private:

    geometry_msgs::msg::Twist twist;

    // Define private helper methods here

    // Declare private member functions for getting and updating twist variable
    geometry_msgs::msg::Twist getTwist() const
    {
        return twist;
    }

    void setTwist(double linearX, double angularZ)
    {
            twist.linear.x = linearX;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = angularZ;
    }



    // Add private methods and member variables as needed
};

#endif // MOTIONCONTROLLER_H