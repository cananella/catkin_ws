#ifndef PID_HPP
#define PID_HPP

class PID {
private:
    double kp, ki, kd;
    double prev_error, integral;
    double dt; 

public:
    PID(double kp, double ki, double kd, double dt);
    double compute(double target, double current);
    void reset();
};

#endif 