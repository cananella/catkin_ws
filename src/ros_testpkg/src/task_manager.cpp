#include "include/task_manager.hpp"
#define Debug false

TaskManager::TaskManager(){
    start_trigger_=nh_.subscribe("/driver_start", 1, &TaskManager::task_Controll, this);
    
}

void TaskManager::task_Controll(const std_msgs::Bool::ConstPtr& msg){
    if(msg){
        task1();
    }
    else std::cout<<" trigger false \n";
}

void TaskManager::task1(){
    TurtleDriver turtle_driver_;
    float line_len=2.0f;
    turtle_driver_.moveLinear(line_len);
    turtle_driver_.moveAngular(PI/2);

    turtle_driver_.moveLinear(line_len);
    turtle_driver_.moveAngular(PI/2+PI/4);
    // turtle_driver_.moveAngular(PI/4);

    turtle_driver_.moveLinear(line_len);
    turtle_driver_.moveAngular(-PI/2);

    turtle_driver_.moveLinear(line_len);
    turtle_driver_.moveAngular(PI/2+PI/4);


    turtle_driver_.moveLinear(line_len);
    turtle_driver_.moveAngular(PI/2);

    turtle_driver_.moveLinear(line_len);

}

void TaskManager::task2(){

}