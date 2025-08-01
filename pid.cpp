// pid.cpp

#include "pid.hpp"

PID::PID(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0) {}

double PID::compute(double error, double dt) {
    integral_ += error * dt;
    double derivative = (error - prev_error_) / dt;
    prev_error_ = error;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

void PID::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
}
