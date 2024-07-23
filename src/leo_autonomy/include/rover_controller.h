#pragma once
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>

struct PidController
{
    double kp;
    double ki;
    double kd;
    double integral;
    double prev_error;
    ros::Time current_time;
    double max_integral;
    double max_output;
    ros::Publisher setpoint_pub;
    ros::Publisher current_pub;
    ros::NodeHandle nh;
    PidController(double kp, double ki, double kd, double max_integral, double max_output, ros::NodeHandle &nh, std::string name);
    double calculate(double setpoint, double current_value, ros::Time current_time);
};

class RoverController
{
  public:
    RoverController(ros::NodeHandle &nh);
    void move_to_pose(geometry_msgs::Pose2D pose, grid_map::GridMap &grid_map, geometry_msgs::Pose2D current_pose);

  private:
    ros::NodeHandle _nh;
    ros::Publisher _cmd_vel_pub;
    geometry_msgs::Twist _twist;
    PidController _pid_controller_angular;
    PidController _pid_controller_linear;
};