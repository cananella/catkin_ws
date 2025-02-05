#include "include/turtle_driver.hpp"
#define Debug false
#define DebugLevel 0


TurtleDriver::TurtleDriver(): running_(true){
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
    subThread_=std::thread(&TurtleDriver::subscriberThread,this);
    sleep(1);
}

TurtleDriver::TurtleDriver(std::string name): running_(true){
    std::string robotname = "/"+name;
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(robotname+"/cmd_vel",10);
    subThread_=std::thread(&TurtleDriver::subscriberThread,this);
    sleep(1);
}

TurtleDriver::~TurtleDriver(){
    running_ = false;
    if (subThread_.joinable()){
        subThread_.join(); 
    }
    std::cout<<"turtle driver stopped \n";
}

void TurtleDriver::subscriberThread(){
    if(robotname.length() == 0) pose_sub_=nh_.subscribe("/turtle1/pose", 2, &TurtleDriver::updatePose, this);
    else pose_sub_=nh_.subscribe(robotname + "/pose", 2, &TurtleDriver::updatePose, this);

    ros::Rate loop_rate(70);
    while(running_){
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void TurtleDriver::updatePose(const turtlesim::Pose::ConstPtr& msg){
    std::unique_lock<std::mutex> lock(mtx);
    turtle_pose_.linear.x=msg->x;
    turtle_pose_.linear.y=msg->y;
    turtle_pose_.angular.z=msg->theta;
    mtx.unlock();

#if DebugLevel >=2
    std::cout<< msg->x << msg->y << msg->theta<<"\n";
#endif
}

void TurtleDriver::move(float linear, float angular){
    geometry_msgs::Twist cmd;
    cmd.linear.x=linear;
    cmd.angular.z=angular;
    cmd_vel_pub_.publish(cmd);
}

void TurtleDriver::moveLinear(float linear, float linear_speed){
    geometry_msgs::Twist cmd;
    float initial_x= turtle_pose_.linear.x;
    float initial_y= turtle_pose_.linear.y;

    float current_x= turtle_pose_.linear.x;
    float current_y= turtle_pose_.linear.y;

#if DebugLevel >=2
    std::cout<<"currnt_x : "<< currnt_x<<"     currnt_y : "<<currnt_y <<" \n";
#endif
    float target_distance = linear;
    float move_distance =0.0f;

    float threshold =0.001f;
    float threshold1 =0.01f;
    float threshold2 = 0.05f;
    cmd.linear.x = linear_speed;

    while(ros::ok() && move_distance < target_distance-threshold){
        current_x= turtle_pose_.linear.x;
        current_y= turtle_pose_.linear.y;

        move_distance = sqrt(pow(current_x - initial_x, 2) + pow(current_y - initial_y, 2));
#if DebugLevel >=1
        std::cout<<"current move distans : "<< turtle_pose_.angular.z << "     target theta : "<< target_theta<<" \n";
#endif
        if(move_distance<target_distance-threshold2) cmd_vel_pub_.publish(cmd);
        else if(move_distance<target_distance-threshold1){
            cmd.linear.x = 0.05f;
            cmd_vel_pub_.publish(cmd);
        }
    }
    cmd.linear.x=0.0f;
    cmd_vel_pub_.publish(cmd);
    std::cout<<"linear move done \n";

}

void TurtleDriver::moveAngular(float angular, float angular_speed){
    geometry_msgs::Twist cmd;
    std::cout<<"sleep 2s \n";
    auto duration= ros::Duration(2.0);
    duration.sleep();

    float initial_theta = turtle_pose_.angular.z;
    float target_theta = initial_theta + angular;

    if (target_theta > M_PI) target_theta -= 2 * M_PI;
    if (target_theta < -M_PI) target_theta += 2 * M_PI;

    cmd.angular.z = (angular > 0) ? angular_speed : -angular_speed;
    bool is_cw_turn = (cmd.angular.z > 0)? false : true ; 

    float threshold =0.001f;

    float threshold1 = PI/2;
    float threshold2 = PI/4;
    float threshold3 = 0.05f;

    while(ros::ok() && std::fabs(turtle_pose_.angular.z - target_theta) > threshold){
        geometry_msgs::Twist current_pos = turtle_pose_;

#if DebugLevel >=1
        std::cout<<"current theta : "<< turtle_pose_.angular.z << "     target theta : "<< target_theta<<" \n";
#endif

        if(std::fabs(current_pos.angular.z - target_theta) > threshold1) cmd_vel_pub_.publish(cmd);

        else if(std::fabs(current_pos.angular.z - target_theta) > threshold2) {
            cmd.angular.z = (is_cw_turn) ? -1.0f : 1.0f;
            cmd_vel_pub_.publish(cmd);
        }

        else if(std::fabs(current_pos.angular.z - target_theta) > threshold3){
            cmd.angular.z = (is_cw_turn) ? -0.1f : 0.1f;
            cmd_vel_pub_.publish(cmd);
        }

    }
    cmd.angular.z=0.0f;
    cmd_vel_pub_.publish(cmd);

    std::cout<<"rotate done \n";
}