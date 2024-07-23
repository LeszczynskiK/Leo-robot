#include "rover_controller.h"

PidController::PidController(double kp, double ki, double kd, double max_integral, double max_output, ros::NodeHandle &nh, std::string name)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->max_integral = max_integral;
    this->max_output = max_output;
    this->current_time = ros::Time::ZERO;
    this->integral = 0;
    this->prev_error = 0;
    this->nh = nh;
    setpoint_pub = nh.advertise<std_msgs::Float64>("/controllers/diff_drive/" + name + "/setpoint", 1);
    current_pub = nh.advertise<std_msgs::Float64>("/controllers/diff_drive/" + name + "/current", 1);
}

double PidController::calculate(double setpoint, double current_value, ros::Time current_time)
{
    double error = setpoint - current_value;
    if (current_time == ros::Time::ZERO)
    {
        this->current_time = current_time;
        return 0;
    }
    double dt = (current_time - this->current_time).toSec();
    this->current_time = current_time;
    integral += error * dt;
    if (integral > max_integral)
        integral = max_integral;
    if (integral < -max_integral)
        integral = -max_integral;
    double derivative = (error - prev_error) / dt;
    prev_error = error;
    double output = kp * error + ki * integral + kd * derivative;
    if (output > max_output)
        output = max_output;
    if (output < -max_output)
        output = -max_output;
    std_msgs::Float64 setpoint_msg;
    setpoint_msg.data = setpoint;
    setpoint_pub.publish(setpoint_msg);
    std_msgs::Float64 current_msg;
    current_msg.data = current_value;
    current_pub.publish(current_msg);
    return output;
}

RoverController::RoverController(ros::NodeHandle &nh) : 
    _pid_controller_angular(1.0, 0.0, 0.0, 1.0, 1.0, nh, "angular"),
    _pid_controller_linear(1.0, 0.0, 0.0, 1.0, 1.0, nh, "linear")
{
    _nh = nh;
    _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>("/controllers/diff_drive/cmd_vel", 1);
}

void RoverController::move_to_pose(geometry_msgs::Pose2D pose, grid_map::GridMap &grid_map, geometry_msgs::Pose2D current_pose)
{
    double angle_to_goal = atan2(pose.y - current_pose.y, pose.x - current_pose.x);
    double angle_diff = angle_to_goal - current_pose.theta;
    if (angle_diff > M_PI)
        angle_diff -= 2 * M_PI;
    if (angle_diff < -M_PI)
        angle_diff += 2 * M_PI;
    grid_map::Position position(pose.x, pose.y);
    for (grid_map::CircleIterator iterator(grid_map, position, 0.25); !iterator.isPastEnd(); ++iterator)
        grid_map.at("leo", *iterator) = 0.99;
    double distance_to_goal = sqrt(pow(pose.x - current_pose.x, 2) + pow(pose.y - current_pose.y, 2));
    if (distance_to_goal > 0.1)
    {
        _twist.linear.x = -_pid_controller_linear.calculate(0, distance_to_goal, ros::Time::now());
        _twist.angular.z = -_pid_controller_angular.calculate(0, angle_diff, ros::Time::now());
        _cmd_vel_pub.publish(_twist);
        return;
    }
}