#ifndef TURTLE_DRIVER_HPP
#define TURTLE_DRIVER_HPP

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <thread>
#include <cmath>
#include <unistd.h>

#define PI 3.14159265358979323846
#define MAX_ANGULAR_SPEED 0.5
#define MAX_LINEAR_SPEED 1.5

class TurtleDriver{
    public:
        TurtleDriver();
        ~TurtleDriver();

        void move(float linear, float angular);
        void moveLinear(float linear  , float linear_speed  = MAX_LINEAR_SPEED);
        void moveAngular(float angular, float angular_speed = MAX_ANGULAR_SPEED);

    private:
        void updatePose(const turtlesim::Pose::ConstPtr& msg);
        void subscriberThread();

        ros::NodeHandle nh_;

        ros::Publisher cmd_vel_pub_;
        ros::Subscriber pose_sub_;

        std::thread subThread_;
        geometry_msgs::Twist turtle_pose_;

        bool running_;

};

#endif