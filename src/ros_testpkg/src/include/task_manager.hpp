#ifndef TASK_MANAGER_HPP
#define TASK_MANAGER_HPP

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Spawn.h>
#include <std_srvs/Empty.h>
#include "turtle_driver.hpp"
#include <thread>

class TaskManager{
    public:
        bool task_flag;
        
        TaskManager();

        void task_Controll(const std_msgs::Bool::ConstPtr& msg);
        void task1(std::string name);
        void task2(std::string name);

        bool kill(std::string name);
        bool clear();
        bool spawn(std::string name, float x=5.5f, float y=5.5f, float theta=0.0);
        
    private:
    
        ros::NodeHandle nh_;
        ros::Subscriber start_trigger_;
        ros::ServiceClient kill_service_;
        ros::ServiceClient clear_service_;
        ros::ServiceClient spawn_service_;

};

#endif