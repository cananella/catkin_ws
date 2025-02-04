#ifndef TASK_MANAGER_HPP
#define TASK_MANAGER_HPP

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "turtle_driver.hpp"
#include <thread>

class TaskManager{
    public:
        TaskManager();

        void task_Controll(const std_msgs::Bool::ConstPtr& msg);
        void task1();
        void task2();
        
        TurtleDriver turtle_driver_;
    private:
    
        ros::NodeHandle nh_;
        ros::Subscriber start_trigger_;
};



#endif