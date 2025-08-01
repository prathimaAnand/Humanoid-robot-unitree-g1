/*
Workflow:
1. Subscribes to odometry data.
2. Computes error between current and desired pose.
3. Uses a PID controller to generate control commands.
4. Publishes velocity commands to make the robot move toward the goal.

Testing:
1. Test with the .txt file
2. Testin with Rviz
3. Tune PID and test with the g1
*/

#include "pid.hpp"  

class PIDControllerNode : public rclcpp::Node {
public:
    PIDControllerNode() : Node("pid_controller"),
        pid_x(1.0, 0.0, 0.1),
        pid_y(1.0, 0.0, 0.1),
        pid_theta(1.0, 0.0, 0.1) {
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PIDControllerNode::odom_callback, this, std::placeholders::_1));
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                std::bind(&PIDControllerNode::control_loop, this));
    }

private:
    // control_loop:
    void control_loop() {
        double error_x = goal_x - curr_x;
        double error_y = goal_y - curr_y;
        double error_theta = normalize_angle(goal_theta - curr_theta);

        double dt = 0.01; // Ideally compute from real time

        double cmd_x = pid_x.compute(error_x, dt);
        double cmd_y = pid_y.compute(error_y, dt);
        double cmd_theta = pid_theta.compute(error_theta, dt);

        // publish to g1
        geometry_msgs::msg::Twist cmd_msg;
        cmd_msg.linear.x = cmd_x;
        cmd_msg.linear.y = cmd_y;
        cmd_msg.angular.z = cmd_theta;
        cmd_pub_->publish(cmd_msg);
    }

    // Normalize angle to [-π, π]
    // double normalize_angle(double angle) {
    //     while (angle > M_PI) angle -= 2.0 * M_PI;
    //     while (angle < -M_PI) angle += 2.0 * M_PI;
    //     return angle;
    // }

    // PIDs
    PID pid_x, pid_y, pid_theta;
};


// class PIDControllerNode : public rclcpp::Node {
// public:
//     PIDControllerNode() : Node("pid_controller") {
//         odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//             "/odom", 10, std::bind(&PIDControllerNode::odom_callback, this, std::placeholders::_1));
//         cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
//         timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
//                 std::bind(&PIDControllerNode::control_loop, this));
//     }

// private:
//     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//         // TODO: Extract and store odometry values
//     }

//     void control_loop() {
//         // TODO: Compute errors, run PID, publish cmd
//     }

//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
// };

