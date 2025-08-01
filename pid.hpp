// pid.hpp

#ifndef PID_HPP
#define PID_HPP

class PID {
public:
    PID(double kp, double ki, double kd);
    double compute(double error, double dt);
    void reset();

private:
    double kp_, ki_, kd_;
    double integral_, prev_error_;
};

#endif // PID_HPP
