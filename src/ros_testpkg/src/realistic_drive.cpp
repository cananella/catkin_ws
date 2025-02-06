#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class TurtleSimController {
public:
    TurtleSimController() {
        ros::NodeHandle nh;

        // 구독자 설정: 목표 속도 (target_linear_velocity, target_angular_velocity)
        sub_target = nh.subscribe("/target_velocity", 10, &TurtleSimController::targetVelocityCallback, this);

        // 발행자 설정: 현재 속도 (/turtle1/cmd_vel)
        pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

        // 변수 초기화
        target_linear_velocity = 0.0;
        target_angular_velocity = 0.0;
        current_linear_velocity = 0.0;
        current_angular_velocity = 0.0;

        last_update_time = ros::Time::now();  // 현재 시간 저장

        ros::Rate rate(30);  // 실행 속도 (Hz), 필요하면 변경 가능

        while (ros::ok()) {
            updateVelocity();
            publishVelocity();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Subscriber sub_target;
    ros::Publisher pub_cmd_vel;

    double target_linear_velocity;
    double target_angular_velocity;
    double current_linear_velocity;
    double current_angular_velocity;

    const double acceleration = 2.0;  // 선형 가속도 (m/s²)
    const double angular_acceleration = 1.2;  // 각속도 가속도 (rad/s²)

    ros::Time last_update_time;  // 이전 시간 저장 변수

    void targetVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        target_linear_velocity = msg->linear.x;
        target_angular_velocity = msg->angular.z;
    }

    void updateVelocity() {
        // 현재 시간 가져오기
        ros::Time current_time = ros::Time::now();
        ros::Duration duration = current_time - last_update_time;
        double dt = duration.toSec();  // 경과 시간 (초 단위)
        
        if (dt > 0) {  // dt가 0보다 클 때만 적용
            // 선속도 점진적 증가/감속
            if (current_linear_velocity < target_linear_velocity) {
                current_linear_velocity += acceleration * dt;
            } else if (current_linear_velocity > target_linear_velocity) {
                current_linear_velocity -= acceleration * dt;
            }

            // 각속도 점진적 증가/감속
            if (current_angular_velocity < target_angular_velocity) {
                current_angular_velocity += angular_acceleration * dt;
            } else if (current_angular_velocity > target_angular_velocity) {
                current_angular_velocity -= angular_acceleration * dt;
            }

            // 이전 업데이트 시간 갱신
            last_update_time = current_time;
        }
    }

    void publishVelocity() {
        geometry_msgs::Twist twist;
        twist.linear.x = current_linear_velocity;
        twist.angular.z = current_angular_velocity;
        pub_cmd_vel.publish(twist);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "realistic_controller");
    TurtleSimController controller;
    return 0;
}
