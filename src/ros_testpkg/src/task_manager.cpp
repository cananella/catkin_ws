#include "include/task_manager.hpp"
#define Debug false

TaskManager::TaskManager(){
    start_trigger_=nh_.subscribe("/driver_start", 10, &TaskManager::task_Controll, this);
    
}

void TaskManager::task_Controll(const std_msgs::Bool::ConstPtr& msg){
    if(msg){
        // TurtleDriver turtle_driver_;
        // task1(turtle_driver_);

    }
    else std::cout<<" trigger false \n";
}

void TaskManager::task1(){
    // turtle_driver_.moveLinear(2);
}

void TaskManager::task2(){

}