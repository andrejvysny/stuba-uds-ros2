void Automatic::timerAutoCmd()
{
    /* double min_dist = 10;
    double min_dist_angle = 0;
    for(double x : x_vect_)
    {
        if (x<min_dist)
        {
            min_dist = x;
            min_dist_angle = angle; 
        }
        
        angle += angle_inc;

    }

    double onPlane = 6.28-angle;
    double fullRot = onPlane + 1.57079633; */

    float e_right = 0;
    float e_front = 0;
    float p_reg = 1.5;
    float p_reg_front = 2.5;

    //RCLCPP_INFO(this->get_logger(), "dist_right: %f", dist_right_);
    //RCLCPP_INFO(this->get_logger(), "dist_left: %f", dist_left_);
    geometry_msgs::msg::Twist msg;

    float side_front = dist_front_right_/2;

    if (state_auto_reg_ == 1 && state_ == 2)
    {
        RCLCPP_INFO(this->get_logger(), "autonomous cmd idee");
        e_right = distance_ - dist_right_;
        e_front = distance_ - dist_front_;

        if(e_front>-0.4 /* && side_front<dist_right_ */)
        {
            //double rot = yaw_ + 

            vel_linear_ = 0.1;
            vel_angular_ = /* p_reg * e_right + */ p_reg_front * (-e_front);
        }
        /* else if(e_front>-0.25 && side_front>dist_right_)
        {
            vel_linear_ = 0.1;
            vel_angular_ = p_reg * e_right + p_reg_front * (e_front);
        } */
        else if(dist_front_right_<dist_right_)
        {
            vel_linear_ = 0.1;
            vel_angular_ = 0.3;

        }
        else
        {
            vel_linear_ = 0.1;
            vel_angular_ = p_reg * e_right;  
        }

        

        RCLCPP_INFO(this->get_logger(), "distance_const: %f", distance_);
        RCLCPP_INFO(this->get_logger(), "dist right: %f", dist_right_);
        RCLCPP_INFO(this->get_logger(), "vel_angular: %f", vel_angular_);

        if(vel_angular_>0.22)
        {
            vel_angular_ = 0.2;
        }

        msg.linear.x = vel_linear_;
        msg.angular.z = vel_angular_;
        publisher_ ->publish(msg);
    }
    
    
}