#include <ncurses.h>
#include <unistd.h>
#include "MotionController.cpp"
#include "MotionActions.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


static const std::string MANUAL_CONTROLLER_TOPIC = std::string("/controller_manual");
static const std::string AUTO_CONTROLLER_TOPIC = std::string("/controller_auto");

class AutoController : public rclcpp::Node
{
public:
    AutoController() : Node("auto_controller")
    {
        RCLCPP_INFO(this->get_logger(), "Created AutoController");
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(AUTO_CONTROLLER_TOPIC, 10);
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&AutoController::scan_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;

    const float safeDistanceOuther = 0.7;
    const float safeDistanceInner = 0.5;
    const float speed_ = 0.4;

    std::vector<float> front, back, left, right;
    std::vector<bool> leftTrace;
    std::vector<bool> rightTrace;

    float linearX = 0.0;
    float angularZ = 0.0;
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd;
        divide_scan(*msg);
        cmd = autoMode(msg);
        auto message = std_msgs::msg::Float64MultiArray();
        message.data = {linearX, angularZ};
        publisher_->publish(message);
    }

    bool isInSafeDistance(const sensor_msgs::msg::LaserScan::SharedPtr msg, float safeDistance = 0.7)
    {
        for (float range : msg->ranges)
        {
            if (range != 0.0 && range <= safeDistance)
            {
                return true;
            }
        }
        return false;
    }

    bool isInSafeDistanceInRange(std::vector<float> ranges)
    {
        for (float range : ranges)
        {
            if (range != 0.0 && range <= safeDistanceInner)
            {
                return true;
            }
        }
        return false;
    }

    float getMiddleValue(const std::vector<float> &values)
    {
        if (values.empty())
        {
            throw std::runtime_error("Vector is empty");
        }
        int middleIndex = values.size() / 2;

        return values.at(middleIndex);
    }

    std::vector<float> selectMiddlePointsInRange(const std::vector<float> ranges, int rangePadding = 4)
    {
        int middleIndex = ranges.size() / 2;
        if (ranges.size() % 2 == 0)
        {
            // Even number of elements, select the first of the two middle indices
            middleIndex--;
        }

        int startIndex = middleIndex - rangePadding;
        int endIndex = middleIndex + rangePadding;
        std::vector<float> selectedPoints;

        for (int i = startIndex; i <= endIndex; i++)
        {
            selectedPoints.push_back(ranges.at(i));
        }

        return selectedPoints;
    }

    float getAverage(const std::vector<float>& numbers)
    {
        if (numbers.empty())
        {
            throw std::runtime_error("Vector is empty");
        }
        float sum = 0.0;
        for (float number : numbers)
        {
            sum += number;
        }
        return sum / numbers.size();
    }

    void logLastRanges(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {

        leftTrace.push_back(isInSafeDistanceInRange(selectMiddlePointsInRange(left)));
        rightTrace.push_back(isInSafeDistanceInRange(selectMiddlePointsInRange(right)));

        if (leftTrace.size() > 50)
        {
            leftTrace.erase(leftTrace.begin());
        }

        if (rightTrace.size() > 50)
        {
            rightTrace.erase(rightTrace.begin());
        }
    }

    bool isMoreThan80PercentTrue(const std::vector<bool> &boolVector)
    {
        int trueCount = 0;
        for (bool value : boolVector)
        {
            if (value)
            {
                trueCount++;
            }
        }
        return trueCount > boolVector.size() * 0.6;
    }

    geometry_msgs::msg::Twist autoMode(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        geometry_msgs::msg::Twist cmd;
        divide_scan(*msg);
        logLastRanges(msg);

        RCLCPP_INFO(this->get_logger(), "RIGHT: %d, LEFT: %d, FRONT: %d, BACK: %d, RIGHTTRACE: %d, LEFTTRACE: %d, LEFTSIZE: %d, RIGHTSIZE: %d", 
        (int)isInSafeDistanceInRange(selectMiddlePointsInRange(right)), (int)isInSafeDistanceInRange(selectMiddlePointsInRange(left)), (int)isInSafeDistanceInRange(selectMiddlePointsInRange(front)), (int)isInSafeDistanceInRange(selectMiddlePointsInRange(back)),
        (int)isMoreThan80PercentTrue(rightTrace), (int)isMoreThan80PercentTrue(leftTrace),
        leftTrace.size(), rightTrace.size()
        );


        if (isInSafeDistance(msg))
        {
            
            // If there is an obstacle in front, rotate right
            if(isInSafeDistanceInRange(selectMiddlePointsInRange(front))){
                cmd.linear.x = 0.0;
                cmd.angular.z = speed_;
            }else if(!isInSafeDistanceInRange(selectMiddlePointsInRange(front)) && isInSafeDistanceInRange(selectMiddlePointsInRange(right))){

                    if (getAverage(selectMiddlePointsInRange(right,1)) < safeDistanceInner)
                    {
                        cmd.linear.x = speed_;
                        cmd.angular.z = speed_/2;
                    }else{
                        cmd.linear.x = speed_;
                        cmd.angular.z = -speed_/2;
                    }
                    RCLCPP_INFO(this->get_logger(), "WORKING FOLLOW RIGHT");
            }else{
                cmd.linear.x = speed_;
                cmd.angular.z = 0.0;
            }


            /*if (
                !isInSafeDistanceInRange(selectMiddlePointsInRange(front)) &&
                !isInSafeDistanceInRange(selectMiddlePointsInRange(left)) &&
                isInSafeDistanceInRange(selectMiddlePointsInRange(right)))
            {
                
                if (getLowestNumber(selectMiddlePointsInRange(right)) < safeDistanceBorder)
                {
                    cmd.linear.x = speed_;
                    cmd.angular.z = 0.1;
                }else{
                    cmd.linear.x = speed_;
                    cmd.angular.z = -0.1;
                }

                RCLCPP_INFO(this->get_logger(), "NO FRONT RIGHT");
            }
            else if (
                !isInSafeDistanceInRange(selectMiddlePointsInRange(front)) &&
                isInSafeDistanceInRange(selectMiddlePointsInRange(left)) &&
                !isInSafeDistanceInRange(selectMiddlePointsInRange(right)))
            {
                cmd.linear.x = speed_;
                cmd.angular.z = 0.0;

                if (getLowestNumber(selectMiddlePointsInRange(right)) < safeDistanceBorder)
                {
                    cmd.linear.x = speed_;
                    cmd.angular.z = -0.1;
                }else{
                    cmd.linear.x = speed_;
                    cmd.angular.z = 0.1;
                }

                RCLCPP_INFO(this->get_logger(), "NO FRONT LEFT");
            }
            else if (
                !isInSafeDistanceInRange(selectMiddlePointsInRange(front)) && !isInSafeDistanceInRange(selectMiddlePointsInRange(right)) && !isInSafeDistanceInRange(selectMiddlePointsInRange(left)) &&
                isMoreThan80PercentTrue(rightTrace) &&
                !isMoreThan80PercentTrue(leftTrace))
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = -speed_; // Rotate right
                RCLCPP_INFO(this->get_logger(), "ROTATE RIGHT");
            }
            else if (
                !isInSafeDistanceInRange(selectMiddlePointsInRange(front)) && !isInSafeDistanceInRange(selectMiddlePointsInRange(right)) && !isInSafeDistanceInRange(selectMiddlePointsInRange(left)) &&
                isMoreThan80PercentTrue(leftTrace) &&
                !isMoreThan80PercentTrue(rightTrace))
            {
                cmd.linear.x = 0.0;
                cmd.angular.z = speed_; // Rotate left
                RCLCPP_INFO(this->get_logger(), "ROTATE LEFT");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "FALLBACK ACTION");
                cmd.linear.x = 0.0;
                cmd.angular.z = speed_;
            }*/
        }
        else
        {
            cmd.linear.x = speed_;
            cmd.angular.z = 0.0;
        }

        linearX = cmd.linear.x;
        angularZ = cmd.angular.z;
        return cmd;
    }

    void divide_scan(const sensor_msgs::msg::LaserScan &scan)
    {

        int num_measurements = scan.ranges.size();
        float angle = scan.angle_min;

        front.clear();
        right.clear();
        back.clear();
        left.clear();

        std::vector<float> frontStart;
        std::vector<float> frontEnd;

        for (int i = 0; i < num_measurements; ++i, angle += scan.angle_increment)
        {
            float norm_angle = fmod(angle + 2 * M_PI, 2 * M_PI);

            if (norm_angle <= M_PI / 4)
                frontStart.push_back(scan.ranges[i]);
            else if (norm_angle > 7 * M_PI / 4)
                frontEnd.push_back(scan.ranges[i]);

            else if (norm_angle > M_PI / 4 && norm_angle <= 3 * M_PI / 4)
                left.push_back(scan.ranges[i]);

            else if (norm_angle > 3 * M_PI / 4 && norm_angle <= 5 * M_PI / 4)
                back.push_back(scan.ranges[i]);
        
            else if (norm_angle > 5 * M_PI / 4 && norm_angle <= 7 * M_PI / 4)
                right.push_back(scan.ranges[i]);
        }

        std::reverse(frontStart.begin(), frontStart.end());
        std::reverse(frontEnd.begin(), frontEnd.end());

        front.insert(front.end(), frontStart.begin(), frontStart.end());
        front.insert(front.end(), frontEnd.begin(), frontEnd.end());
    }

  
    float getLowestNumber(const std::vector<float>& numbers)
    {
        float lowestNumber = std::numeric_limits<float>::max();
        for (float number : numbers)
        {
            if (number != 0.0 && number < lowestNumber)
            {
                lowestNumber = number;
            }
        }
        return lowestNumber;
    }
};