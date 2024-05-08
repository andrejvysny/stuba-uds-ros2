#include <ncurses.h>
#include <unistd.h>
#include "MotionController.cpp"
#include "MotionActions.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

static const std::string MANUAL_CONTROLLER_TOPIC = std::string("/controller_manual");
static const std::string AUTO_CONTROLLER_TOPIC = std::string("/controller_auto");


class AutoController: public rclcpp::Node
{
public:

    AutoController() : Node("auto_controller")
    {
        RCLCPP_INFO(this->get_logger(),"Created AutoController");
        
        publisher_ = this->create_publisher<std_msgs::msg::Int32>(AUTO_CONTROLLER_TOPIC, 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&AutoController::timer_callback, this)
        );


        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&AutoController::scan_callback, this, std::placeholders::_1));

        
        command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    }



private:

    const float safeDistance = 0.5;
    const float speed_ = 0.2;

    std::vector<float> front, back, left, right;


    bool isInSafeDistance(const sensor_msgs::msg::LaserScan::SharedPtr msg, float safeDistance = 0.45) {
        for (float range : msg->ranges) {
            if (range != 0.0 && range <= safeDistance) {
                
                RCLCPP_INFO(this->get_logger(), "TRUE isInSafeDistance: %f", range);
                return true;
            }
            RCLCPP_INFO(this->get_logger(), "\tFALSE isInSafeDistance: %f", range);

        }
        return false;
    }

    bool isInSafeDistanceInRange(std::vector<float> ranges) {
        for (float range : ranges) {
            if (range != 0.0 && range <= safeDistance) {
                RCLCPP_INFO(this->get_logger(), "TRUE isInSafeDistanceInRange: %f", range);
                return true;
            }
            RCLCPP_INFO(this->get_logger(), "\tFALSE isInSafeDistanceInRange: %f", range);
        }
        return false;
    }


    float getMiddleValue(const std::vector<float>& values) {
        if (values.empty()) {
            throw std::runtime_error("Vector is empty");
        }
        int middleIndex = values.size() / 2;
        RCLCPP_INFO(this->get_logger(), "getMiddleValue");

        return values.at(middleIndex);
    }

    std::vector<float> selectMiddlePointsInRange(const std::vector<float> ranges) {
        int middleIndex = ranges.size() / 2;
        if (ranges.size() % 2 == 0) {
            // Even number of elements, select the first of the two middle indices
            middleIndex--;
        }

        int startIndex = middleIndex - 5;
        int endIndex = middleIndex + 5;

        RCLCPP_INFO(this->get_logger(), "MIDDLE INDEX: %d", middleIndex);

        std::vector<float> selectedPoints;
        
        RCLCPP_INFO(this->get_logger(), "selectMiddlePointsInRange before for");

        for (int i = startIndex; i <= endIndex; i++) {
            selectedPoints.push_back(ranges.at(i));
        }
        RCLCPP_INFO(this->get_logger(), "selectMiddlePointsInRange after for");

        return selectedPoints;
    }


    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        geometry_msgs::msg::Twist cmd;
        divide_scan(*msg);
        cmd = autoMode(msg);
        command_publisher_->publish(cmd);
    }




    geometry_msgs::msg::Twist autoMode(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

        geometry_msgs::msg::Twist cmd;

        RCLCPP_INFO(this->get_logger(), "before devide scan");

        divide_scan(*msg);

        RCLCPP_INFO(this->get_logger(), "Front: %d, Back: %d, Left: %d, Right: %d;", front.size(), back.size(), left.size(), right.size());


        RCLCPP_INFO(this->get_logger(), "after devide scan");


        if (isInSafeDistance(msg)) {

            RCLCPP_INFO(this->get_logger(), "IN SAVE DISTANCE IF");

            if(
                !isInSafeDistanceInRange(selectMiddlePointsInRange(front)) 
                && (
                    isInSafeDistanceInRange(selectMiddlePointsInRange(right)) 
                    || 
                    isInSafeDistanceInRange(selectMiddlePointsInRange(left))
                    )
            ){
                cmd.linear.x = speed_;
                cmd.angular.z = 0.0;
            }else{
                cmd.linear.x = 0.0;
                cmd.angular.z = speed_;
            }
            
        } else {
            cmd.linear.x = speed_;
            cmd.angular.z = 0.0;
        }

        return cmd;
    }






    int determineLowestRange()
    {
        float min_range = std::numeric_limits<float>::max();
        int range = 0;

        if (!front.empty())
        {
            for (float value : front)
            {
                if (value != 0.0 && value < min_range)
                {
                    min_range = value;
                    range = 1; // Front
                }
            }
        }

        if (!back.empty())
        {
            for (float value : back)
            {
                if (value != 0.0 && value < min_range)
                {
                    min_range = value;
                    range = 2; // Back
                }
            }
        }

        if (!left.empty())
        {
            for (float value : left)
            {
                if (value != 0.0 && value < min_range)
                {
                    min_range = value;
                    range = 3; // Left
                }
            }
        }

        if (!right.empty())
        {
            for (float value : right)
            {
                if (value != 0.0 && value < min_range)
                {
                    min_range = value;
                    range = 4; // Right
                }
            }
        }

        return range;
    }

    void divide_scan(const sensor_msgs::msg::LaserScan& scan) {

        int num_measurements = scan.ranges.size();
        float angle = scan.angle_min;

          front.clear();
            right.clear();
            back.clear();   
            left.clear();         

        for (int i = 0; i < num_measurements; ++i, angle += scan.angle_increment) {
            // Normalize angle to be within [0, 2*pi]
            float norm_angle = fmod(angle + 2 * M_PI, 2 * M_PI);
   
            // Front: -45 to +45 degrees around the x-axis
            if (norm_angle <= M_PI / 4 || norm_angle > 7 * M_PI / 4)
                front.push_back(scan.ranges[i]);
            // Right: 45 to 135 degrees
            else if (norm_angle > M_PI / 4 && norm_angle <= 3 * M_PI / 4)
                right.push_back(scan.ranges[i]);
            // Back: 135 to 225 degrees
            else if (norm_angle > 3 * M_PI / 4 && norm_angle <= 5 * M_PI / 4)
                back.push_back(scan.ranges[i]);
            // Left: 225 to 315 degrees
            else if (norm_angle > 5 * M_PI / 4 && norm_angle <= 7 * M_PI / 4)
                left.push_back(scan.ranges[i]);
        }

    }


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;



    void timer_callback()
    {
        auto message = std_msgs::msg::Int32();
        message.data = MotionActions::ACTION_NONE;
        publisher_->publish(message);
    }
 

  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received scan: range size=%lu", msg->ranges.size());
        // Optionally, log some ranges
        if (!msg->ranges.empty()) {
            RCLCPP_INFO(this->get_logger(), "First range: %f, Last range: %f",
                        msg->ranges.front(), msg->ranges.back());
        }
    }


    void findClosestPoint(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
        float closestRange = std::numeric_limits<float>::max();
        int closestIndex = -1;

        for (int i = 0; i < msg->ranges.size(); i++)
        {
            if (msg->ranges[i] < closestRange)
            {
                closestRange = msg->ranges[i];
                closestIndex = i;
            }
        }

        if (closestIndex != -1)
        {
            RCLCPP_INFO(this->get_logger(), "Closest point: index=%d, range=%f", closestIndex, closestRange);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "No points found in LaserScan message");
        }
    }


    
};