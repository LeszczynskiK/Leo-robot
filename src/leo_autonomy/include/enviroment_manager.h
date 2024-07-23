#pragma once
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/TypeDefs.hpp"
#include "obstacle_spawner.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <cstdlib>
#include <ctime>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>

class EnviromentCreator
{
  public:
    EnviromentCreator(ros::NodeHandle &nh, unsigned int obstacle_count);
    geometry_msgs::Pose2D get_leo_pose();
    grid_map::GridMap *get_grid_map();
    bool environment_ready();

  private:
    void wait_until_sim_time_starts();
    void init(unsigned int obstacle_count);
    void create_grid_map();
    void generate_line_obstacles();
    void generate_circle_obstacles(grid_map::Position position);
    void update_leo_pos();
    void add_circle_obstacle(double radius, double alpha, grid_map::Position position);
    void update_gazebo();
    double _map_resolution = 0.1;
    double _map_width = 15.0;
    ros::NodeHandle _nh;
    ObstacleSpawner _obstacle_spawner;
    ros::ServiceClient _unpause_gz_client;
    ros::ServiceClient _pause_gz_client;
    gazebo_msgs::ModelStates _model_states;
    ros::Publisher _grid_map_pub;
    grid_map::GridMap _grid_map;
    ros::Timer _timer;
    ros::Subscriber _model_states_sub;
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    geometry_msgs::TransformStamped _tf;
    bool _environment_ready = false;
};
