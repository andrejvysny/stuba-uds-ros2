#include <ncurses.h>
#include <unistd.h>
#include "MotionController.cpp"
#include "MotionActions.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
static const std::string MANUAL_CONTROLLER_TOPIC = std::string("/controller_manual");
static const std::string AUTO_CONTROLLER_TOPIC = std::string("/controller_auto");




class ManualController: public rclcpp::Node
{
public:

    ManualController() : Node("manual_controller")
    {
        RCLCPP_INFO(this->get_logger(),"Created ManualController, LISTENING ON KEYS w, a, s, d, q, x");
        
        publisher_ = this->create_publisher<std_msgs::msg::Int32>(MANUAL_CONTROLLER_TOPIC, 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ManualController::timer_callback, this)
        );
        init_keyboard();
    }

    ~ManualController()
    {
        close_keyboard();
    }



private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

    void timer_callback()
    {
        char key = read_key();
        if (key != EOF) {
            auto message = std_msgs::msg::Int32();
            message.data = mapping(key);
            publisher_->publish(message);
        }
    }
 
    void init_keyboard()
    {
        initscr();
        cbreak();
        noecho();
        nodelay(stdscr, TRUE);
    }

    int read_key()
    {
        int ch = getch();
        if (ch != ERR)
        {
            flushinp(); // Clear out buffer to avoid multiple commands on single press
        }
        return ch;
    }

    void close_keyboard()
    {
        endwin();
    }


    int mapping(int key)
    {
        switch (key)
        {
            case 'w':
                return MotionActions::ACTION_INCREMENT_LINEAR_SPEED;
                break;
            case 'a':
                return MotionActions::ACTION_INCREMENT_ANGULAR_SPEED;
                break;
            case 's':
                return MotionActions::ACTION_DECREMENT_LINEAR_SPEED;
                break;
            case 'd':
                return MotionActions::ACTION_DECREMENT_ANGULAR_SPEED;
                break;
            case 'q':
                return MotionActions::ACTION_STOP;
                break;
            default:
                return MotionActions::ACTION_NONE;
                break;
        }
    }
};