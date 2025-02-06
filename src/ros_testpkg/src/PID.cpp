#include "include/PID.hpp"
#include <iostream>

PID::PID(double kp, double ki, double kd, double dt)
    : kp(kp), ki(ki), kd(kd), dt(dt), prev_error(0.0), integral(0.0) {}

double PID::compute(double target, double current) {
    double error = target - current;
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    
    double output = kp * error + ki * integral + kd * derivative;
    prev_error = error;
    
    return output;
}

void PID::reset() {
    prev_error = 0.0;
    integral = 0.0;
}
