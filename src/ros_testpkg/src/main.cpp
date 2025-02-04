#include "include/task_manager.hpp"
#include <ros/ros.h>

int main(int argc, char ** argv) {
    ros::init(argc,argv,"turtle_task_node");
    TaskManager task_manager;
    
    ros::Rate loop_rate(10);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    

    return 0;
}