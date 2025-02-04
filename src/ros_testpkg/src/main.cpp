#include "include/task_manager.hpp"
#include <ros/ros.h>

int main(int argc, char ** argv) {
    ros::init(argc,argv,"turtle_task_node");
    TaskManager task_manager;
    
    // ros::spinOnce();
    std::cout<<"================";
    task_manager.turtle_driver_.~TurtleDriver();
    // ros::shutdown();
    

    return 0;
}