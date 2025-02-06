#include "include/turtle_driver.hpp"


TurtleDriver::TurtleDriver(): running_(true){
    sleep(0.1);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/realistic/cmd_vel",10);
    subThread_=std::thread(&TurtleDriver::subscriberThread,this);
    sleep(1);
}

TurtleDriver::TurtleDriver(std::string name): running_(true){
    robotname = "/"+name;
    sleep(0.1);
    // cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(robotname+"/cmd_vel",10);
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
    if(robotname.length() == 0) 
        pose_sub_=nh_.subscribe("/turtle1/pose", 2, &TurtleDriver::updatePose, this);
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
    std::cout<<"-------------------- thread run ";
    std::cout<< msg->x <<"  " << msg->y<<"  " << msg->theta<<"\n";
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
        std::cout<<"current move distans : "<< move_distance << "     target theta : "<< linear<<" \n";
#endif
        if(move_distance<target_distance-threshold2) cmd_vel_pub_.publish(cmd);
        else if(move_distance<target_distance-threshold1){
            cmd.linear.x = 0.05f;
            cmd_vel_pub_.publish(cmd);
        }
        else if(move_distance<target_distance-threshold){
            cmd.linear.x = 0.01f;
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

    
    float target_velocity  = (angular > 0) ? angular_speed : -angular_speed;
    bool is_cw_turn = (target_velocity > 0)? false : true ; 

    float threshold =0.001f;

    float threshold1 = PI/2;
    float threshold2 = PI/4;
    float threshold3 = 0.05f;

    while(ros::ok() && std::fabs(turtle_pose_.angular.z - target_theta) > threshold){
        geometry_msgs::Twist current_pos = turtle_pose_;

#if DebugLevel >=1
        std::cout<<"current theta : "<< turtle_pose_.angular.z << "     target theta : "<< target_theta<<" \n";
#endif

        if(std::fabs(current_pos.angular.z - target_theta) > threshold1) target_velocity = angular_speed;

        else if(std::fabs(current_pos.angular.z - target_theta) > threshold2) {
            target_velocity = (is_cw_turn) ? -1.0f : 1.0f;
        }

        else if(std::fabs(current_pos.angular.z - target_theta) > threshold3){
            target_velocity = (is_cw_turn) ? -0.1f : 0.1f;
        }

        else if(std::fabs(current_pos.angular.z - target_theta) > threshold){
            target_velocity = (is_cw_turn) ? -0.08f : 0.08f;
            
        }else target_velocity=0.0f;

        cmd.angular.z=target_velocity;
        cmd_vel_pub_.publish(cmd);

    }
    cmd.angular.z=0.0f;
    cmd_vel_pub_.publish(cmd);

    std::cout<<"rotate done \n";
}

void TurtleDriver::moveArc(float target_angular, float linear_speed ,float angular_speed ){
    geometry_msgs::Twist cmd;
    geometry_msgs::Twist current_pos = turtle_pose_;

    float initial_theta = turtle_pose_.angular.z;
    float target_theta = initial_theta + target_angular;
    float total_theta = initial_theta;
    if (target_theta > 3*M_PI) target_theta -= 3 * M_PI;
    if (target_theta < 0) target_theta += 2 * M_PI;
    current_pos.angular.z = (turtle_pose_.angular.z < 0)? (turtle_pose_.angular.z + 2*M_PI) : turtle_pose_.angular.z;

    std::cout<<"current theta : "<< current_pos.angular.z << "     target theta : "<< target_theta<<" \n";
    
    std::cout<<"sleep 2s \n";
    auto duration= ros::Duration(2.0);
    duration.sleep();

    float threshold =0.1f;
    float threshold1 = PI/2;
    float threshold2 = PI/4;
    float threshold3 = 0.4f;

    cmd.linear.x = linear_speed;
    cmd.angular.z = angular_speed;

    current_pos.angular.z = (turtle_pose_.angular.z < 0)? (turtle_pose_.angular.z + 2*M_PI) : turtle_pose_.angular.z;
    
    while(ros::ok() && std::fabs(total_theta - target_theta) > threshold){
        float prev_theta = total_theta;
        float current_theta = (turtle_pose_.angular.z < 0)? (turtle_pose_.angular.z + 2 * M_PI) : turtle_pose_.angular.z;    
        float delta_theta = current_theta - prev_theta;
        if (delta_theta > M_PI) {
            delta_theta -= 2 * M_PI;
        } else if (delta_theta < -M_PI) {
            delta_theta += 2 * M_PI;
        }

        total_theta += delta_theta; 

#if DebugLevel >=1
        std::cout<<"current theta : "<< total_theta << "     target theta : "<< target_theta<<" \n";
#endif

        if(std::fabs(total_theta - target_theta) > threshold1) cmd_vel_pub_.publish(cmd);

        else if(std::fabs(total_theta - target_theta) > threshold2) {
            cmd.linear.x = linear_speed/2;
            cmd.angular.z = angular_speed/2;
            cmd_vel_pub_.publish(cmd);
        }

        else if(std::fabs(total_theta - target_theta) > threshold3){
            cmd.linear.x = linear_speed/4;
            cmd.angular.z = angular_speed/4;
            cmd_vel_pub_.publish(cmd);
        }

        else if(std::fabs(total_theta - target_theta) > threshold){
            cmd.linear.x = linear_speed/5;
            cmd.angular.z = angular_speed/5;
            cmd_vel_pub_.publish(cmd);
        }
    }
    cmd.angular.z=0.0f;
    cmd.linear.x =0.0f;
    cmd_vel_pub_.publish(cmd);

}


void TurtleDriver::moveArcWithPID(float target_angular, float linear_speed ,float angular_speed ){
    geometry_msgs::Twist cmd;
    geometry_msgs::Twist current_pos = turtle_pose_;

    float initial_theta = turtle_pose_.angular.z;
    float target_theta = initial_theta + target_angular;
    float total_theta = initial_theta;
    if (target_theta > 3*M_PI) target_theta -= 3 * M_PI;
    if (target_theta < 0) target_theta += 2 * M_PI;
    current_pos.angular.z = (turtle_pose_.angular.z < 0)? (turtle_pose_.angular.z + 2*M_PI) : turtle_pose_.angular.z;

    std::cout<<"current theta : "<< current_pos.angular.z << "     target theta : "<< target_theta<<" \n";
    
    std::cout<<"sleep 2s \n";
    auto duration= ros::Duration(2.0);
    duration.sleep();

    float threshold =0.1f;
    float threshold1 = PI/2;
    float threshold2 = PI/4;
    float threshold3 = 0.4f;
    float target_linear_velocity;
    float target_angular_velocity;
    target_linear_velocity = linear_speed;
    target_angular_velocity = angular_speed;

    current_pos.angular.z = (turtle_pose_.angular.z < 0)? (turtle_pose_.angular.z + 2*M_PI) : turtle_pose_.angular.z;
    
    while(ros::ok() && std::fabs(total_theta - target_theta) > threshold){
        float prev_theta = total_theta;
        float current_theta = (turtle_pose_.angular.z < 0)? (turtle_pose_.angular.z + 2 * M_PI) : turtle_pose_.angular.z;    
        float delta_theta = current_theta - prev_theta;
        if (delta_theta > M_PI) {
            delta_theta -= 2 * M_PI;
        } else if (delta_theta < -M_PI) {
            delta_theta += 2 * M_PI;
        }

        total_theta += delta_theta; 

#if DebugLevel >=1
        std::cout<<"current theta : "<< total_theta << "     target theta : "<< target_theta<<" \n";
#endif

        if(std::fabs(total_theta - target_theta) > threshold1) {
            target_linear_velocity = linear_speed;
            target_angular_velocity = angular_speed;
        }

        else if(std::fabs(total_theta - target_theta) > threshold2) {
            target_linear_velocity = linear_speed/2;
            target_angular_velocity = angular_speed/2;
        }

        else if(std::fabs(total_theta - target_theta) > threshold3){
            target_linear_velocity = linear_speed/4;
            target_angular_velocity = angular_speed/4;
        }

        else if(std::fabs(total_theta - target_theta) > threshold){
            target_linear_velocity = linear_speed/5;
            target_angular_velocity = angular_speed/5;
        }

        else{
            target_linear_velocity =0.0f;
            target_angular_velocity =0.0f;
        }

    cmd.angular.z=target_angular_velocity;
    cmd.linear.x =target_linear_velocity;
        cmd_vel_pub_.publish(cmd);
    }
    cmd.angular.z=0.0f;
    cmd.linear.x =0.0f;
    cmd_vel_pub_.publish(cmd);

}
