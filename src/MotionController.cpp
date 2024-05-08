#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include "geometry_msgs/msg/twist.hpp"


class MotionController {
public:
    MotionController() {
        stop();
        last_time_ = rclcpp::Clock().now();
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

        totalDistance += calculateTraveledDistance(linearX, angularZ);
        return twist;
    }

    void setLinearSpeed(double speed)
    {
        linearX = speed;
    }

    void setAngularSpeed(double speed)
    {
        angularZ = speed;
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

    double getTotalDistance()
    {
        return totalDistance;
    }

private:

    geometry_msgs::msg::Twist twist;

    double linearX = 0.0;
    double angularZ = 0.0;

    rclcpp::Time last_time_;

    double totalDistance = 0.0;

    double calculateTraveledDistance(double linear_speed, double angular_speed) 
    {
        double current_time = rclcpp::Clock().now().seconds();
        double time_difference = current_time - last_time_.seconds();
        last_time_ = rclcpp::Clock().now();

        double distance = linear_speed * time_difference;

        // If angular_speed is zero, the motion is in a straight line
        if (angular_speed == 0.0) {
            return distance;
        } else {
            // For non-zero angular speed, calculate the arc length
            double radius = linear_speed / angular_speed;
            double arc_length = angular_speed * radius * time_difference;
            return arc_length;
        }
    }

};

#endif // MOTIONCONTROLLER_H