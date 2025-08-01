/*
PID controller for error correction between the target pose and the odometry data from robot (robot pose)

Output from odometry data:
foot_position_body
foot_raise_height
*/

double kp = 1.0, ki = 0.0, kd = 0.1;
double integral = 0.0, prev_error = 0.0, dt = 0.01;
double integral_x, prev_error_x, integral_y, prev_error_y, integral_theta, prev_error_theta;

double pid_control(double error, double& integral, double& prev_error, double dt) {
    integral += error * dt;
    double derivative = (error - prev_error) / dt;
    prev_error = error;
    return kp * error + ki * integral + kd * derivative;
}

// dummy - TODO: correct a/c to odom data
// Current position and orientation
double curr_x, curr_y, curr_z, curr_theta;

// Target
double goal_x, goal_y, goal_z, goal_theta;

// Position Error
double error_x = goal_x - curr_x;
double error_y = goal_y - curr_y;
double error_z = goal_z - curr_z;
double error_theta = goal_theta - curr_theta;  // NOTE: Be careful with angle wraparound

// TODO: Implement in a Class
double Twist_cmd;
x = pid_control(error_x, integral_x, prev_error_x, dt);
y = pid_control(error_y, integral_y, prev_error_y, dt);
z = pid_control(error_theta, integral_theta, prev_error_theta, dt);

// Next: Publish to /cmd_vel or equivalent topic
