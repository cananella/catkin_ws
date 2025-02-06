#include "include/task_manager.hpp"
#include "include/turtle_driver.hpp"
#include <ros/ros.h>
#include <thread>
    

bool move_start =false;

void move_Idle(TurtleDriver& turtle_driver_2){
    while(ros::ok()){
        if(move_start){
            turtle_driver_2.moveArc(2*M_PI,2.0f,1.5f);
            break;
        }
    }
   
}


void move_Odd(TurtleDriver& turtle_driver_){
    while(ros::ok()){
        if(move_start){
            turtle_driver_.moveArcWithPID(2*M_PI,2.0f,1.5f);
            break;
        }
    }
   
}


int main(int argc, char ** argv) {
    ros::init(argc,argv,"turtle_task_node");
    TurtleDriver turtle_driver_;
    TurtleDriver turtle_driver_2("turtle2");
    TaskManager task_manager_;
    task_manager_.spawn("turtle2");
    std::thread turtle2_thread=std::thread(move_Idle,std::ref(turtle_driver_2));
    std::thread turtle1_thread=std::thread(move_Odd,std::ref(turtle_driver_));
    sleep(2.0);
    move_start =true;
    if (turtle1_thread.joinable()&&turtle2_thread.joinable()){
        turtle1_thread.join(); 
        turtle2_thread.join();
    }

    std::cout<<"turn done    \n";
    sleep(2.0);
    
    

    return 0;
}