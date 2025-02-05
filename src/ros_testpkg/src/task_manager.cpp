#include "include/task_manager.hpp"
#include<memory>

TaskManager::TaskManager() : task_flag(true){
    start_trigger_=nh_.subscribe("/driver_start", 1, &TaskManager::task_Controll, this);
    kill_service_ = nh_.serviceClient<turtlesim::Kill>("kill");
    spawn_service_ = nh_.serviceClient<turtlesim::Spawn>("spawn");
    clear_service_ = nh_.serviceClient<std_srvs::Empty>("clear");
    
}

void TaskManager::task_Controll(const std_msgs::Bool::ConstPtr& msg){
    if(msg){
        std::string robot1_name="turtle1";
        std::string robot2_name="turtle2";

        task1(robot1_name);
        kill(robot1_name);
        clear();
        spawn(robot2_name);
        sleep(2.0);
        task2(robot2_name);

        std::cout<<"task done \n";
        task_flag = false;
    }
    else {
        std::cout<<" trigger false \n";
        task_flag =false;
    }
}

void TaskManager::task1(std::string name){
    std::shared_ptr<TurtleDriver> turtle_driver_ = std::make_shared<TurtleDriver>(name);

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

}

void TaskManager::task2(std::string name){
    std::shared_ptr<TurtleDriver> turtle_driver_ = std::make_shared<TurtleDriver>(name);

    turtle_driver_->moveArc(2*M_PI,2.0f,1.5f);

    turtle_driver_->moveArc(M_PI,2.0f,-1.5f);

    turtle_driver_->moveArc(M_PI,2.0f,-3.0f);
    
    turtle_driver_->moveArc(M_PI,2.0f,3.0f);
    
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