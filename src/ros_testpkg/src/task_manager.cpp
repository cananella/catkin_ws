#include "include/task_manager.hpp"
#define Debug false

TaskManager::TaskManager() : task_flag(true){
    start_trigger_=nh_.subscribe("/driver_start", 1, &TaskManager::task_Controll, this);
    kill_service_ = nh_.serviceClient<turtlesim::Kill>("kill");
    spawn_service_ = nh_.serviceClient<turtlesim::Spawn>("spawn");
    clear_service_ = nh_.serviceClient<std_srvs::Empty>("clear");
    
}

void TaskManager::task_Controll(const std_msgs::Bool::ConstPtr& msg){
    if(msg){
        task1();
        kill("turtle1");
        clear();
        spawn("turtle2");

        std::cout<<"task done \n";
        task_flag = false;
    }
    else {
        std::cout<<" trigger false \n";
        task_flag =false;
    }
}

void TaskManager::task1(){
    TurtleDriver* turtle_driver_ = new TurtleDriver("turtle1");
    float line_len=2.0f;
    turtle_driver_->moveLinear(line_len);
    turtle_driver_->moveAngular(PI/2);

    turtle_driver_->moveLinear(line_len);
    turtle_driver_->moveAngular(PI/2+PI/4);

    turtle_driver_->moveLinear(line_len);
    turtle_driver_->moveAngular(-PI/2);

    turtle_driver_->moveLinear(line_len);
    turtle_driver_->moveAngular(PI/2+PI/4);

    turtle_driver_->moveLinear(line_len);
    turtle_driver_->moveAngular(PI/2);

    turtle_driver_->moveLinear(line_len);

    delete turtle_driver_;

}

void TaskManager::task2(){

}




bool TaskManager::kill(std::string name){
    turtlesim::Kill kill;
    kill.request.name = name;
    if(kill_service_.call(kill)){
        std::cout<< "kill service called \n";
        return true;
    }else return false;

}

bool TaskManager::clear(){
    std_srvs::Empty empty;
    if(clear_service_.call(empty)){
        std::cout<< "clear service called \n";
        return true;
    }else return false;

}

bool TaskManager::spawn(std::string name, float x, float y, float theta){
    turtlesim::Spawn spawn;
    spawn.request.name = name;
    spawn.request.x = x;
    spawn.request.y = y;
    spawn.request.theta = theta;
    if(spawn_service_.call(spawn)){
        std::cout<< "spawn service called \n";
        return true;
    }else return false;

}