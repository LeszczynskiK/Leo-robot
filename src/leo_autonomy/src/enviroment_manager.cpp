#include "enviroment_manager.h"
#include "grid_map_core/GridMap.hpp"
#include <tf2/utils.h>

EnviromentCreator::EnviromentCreator(ros::NodeHandle &nh, unsigned int obstacle_count) : _nh(nh), _obstacle_spawner(nh)
{
    wait_until_sim_time_starts();
    _unpause_gz_client = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    _pause_gz_client = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    _model_states_sub = _nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, [this](const gazebo_msgs::ModelStates::ConstPtr &msg) { _model_states = *msg; });
    init(obstacle_count);
    std_srvs::Empty srv;
    _unpause_gz_client.call(srv);
    _grid_map_pub = _nh.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
    _tf.header.frame_id = "map";
    _tf.child_frame_id = "base_footprint";
    _tf.transform.rotation.w = 1.0;
    create_grid_map();
    _timer = _nh.createTimer(ros::Duration(0.25), [this](const ros::TimerEvent &event) {
        update_leo_pos();
        update_gazebo();
        grid_map_msgs::GridMap _grid_map_msg;
        grid_map::GridMapRosConverter::toMessage(_grid_map, _grid_map_msg);
        _grid_map_msg.info.header.stamp = ros::Time::now();
        _grid_map_pub.publish(_grid_map_msg);
    });
    ROS_INFO("Environment created");
    _environment_ready = true;
}

void EnviromentCreator::wait_until_sim_time_starts()
{
    bool ready = false;
    while (!ready)
    {
        ready = ros::service::exists("/gazebo/spawn_sdf_model", true) &&
                ros::service::exists("/gazebo/unpause_physics", true);
    }
}
void EnviromentCreator::init(unsigned int obstacle_count)
{
    srand(time(nullptr));
    for (unsigned int i = 0; i < obstacle_count; i++)
    {
        geometry_msgs::Pose pose;
        float random_x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX / 12) - 6;
        float random_y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX / 12) - 6;
        if (random_x > -2 && random_x < 0)
            random_x -= 1;
        if (random_x > 0 && random_x < 2)
            random_x += 1;
        if (random_y > -2 && random_y < 0)
            random_y -= 1;
        if (random_y > 0 && random_y < 2)
            random_y += 1;
        pose.position.x = random_x;
        pose.position.y = random_y;
        _obstacle_spawner.spawnBox(pose);
    }
}
void EnviromentCreator::create_grid_map()
{
    while (_model_states.name.empty())
        ros::spinOnce();
    _grid_map.setFrameId("map");
    _grid_map.setGeometry(grid_map::Length(_map_width, _map_width), _map_resolution, grid_map::Position(0, 0));
    _grid_map.add("obstacles", 0.0);
    _grid_map.add("leo", 0.0);
    for (unsigned int i = 0; i < _model_states.name.size(); i++)
    {
        if (_model_states.name[i] == "ground_plane")
            continue;
        if (_model_states.name[i] == "leo")
            continue;
        grid_map::Position position(_model_states.pose[i].position.x, _model_states.pose[i].position.y);
        generate_circle_obstacles(position);
    }
    generate_line_obstacles();
}
void EnviromentCreator::generate_line_obstacles()
{
    double cell_count = _map_width / _map_resolution - 1;
    grid_map::Index start(0, 0);
    grid_map::Index end(0, cell_count);
    for (grid_map::LineIterator iterator(_grid_map, start, end); !iterator.isPastEnd(); ++iterator)
        _grid_map.at("obstacles", *iterator) = 1.0;
    end = grid_map::Index(cell_count, 0);
    for (grid_map::LineIterator iterator(_grid_map, start, end); !iterator.isPastEnd(); ++iterator)
        _grid_map.at("obstacles", *iterator) = 1.0;
    start = grid_map::Index(cell_count, 0);
    end = grid_map::Index(cell_count, cell_count);
    for (grid_map::LineIterator iterator(_grid_map, start, end); !iterator.isPastEnd(); ++iterator)
        _grid_map.at("obstacles", *iterator) = 1.0;
    start = grid_map::Index(0, cell_count);
    for (grid_map::LineIterator iterator(_grid_map, start, end); !iterator.isPastEnd(); ++iterator)
        _grid_map.at("obstacles", *iterator) = 1.0;
}
void EnviromentCreator::generate_circle_obstacles(grid_map::Position position)
{
    add_circle_obstacle(1.0, 0.5, position);
    add_circle_obstacle(0.75, 0.75, position);
    add_circle_obstacle(0.5, 1.0, position);
}

void EnviromentCreator::update_leo_pos()
{
    //clear previous leo position
    for (grid_map::GridMapIterator iterator(_grid_map); !iterator.isPastEnd(); ++iterator)
    {
        if (_grid_map.at("leo", *iterator) == 0.0)
            continue;
        if (_grid_map.at("leo", *iterator) >= 0.9 && _grid_map.at("leo", *iterator) <= 1.0)
            continue;
        _grid_map.at("leo", *iterator) = 0.0;
    }
    for (unsigned int i = 0; i < _model_states.name.size(); i++)
    {
        if (_model_states.name[i] == "leo")
        {
            grid_map::Position position(_model_states.pose[i].position.x, _model_states.pose[i].position.y);
            for (grid_map::CircleIterator iterator(_grid_map, position, 0.25); !iterator.isPastEnd(); ++iterator)
                _grid_map.at("leo", *iterator) = 1.0;
            for (grid_map::CircleIterator iterator(_grid_map, position, 0.5); !iterator.isPastEnd(); ++iterator)
                _grid_map.at("leo", *iterator) = 0.5;
            _tf.header.stamp = ros::Time::now();
            _tf.transform.translation.x = _model_states.pose[i].position.x;
            _tf.transform.translation.y = _model_states.pose[i].position.y;
            _tf.transform.rotation = _model_states.pose[i].orientation;
            _tf_broadcaster.sendTransform(_tf);
        }
    }
}

void EnviromentCreator::add_circle_obstacle(double radius, double alpha, grid_map::Position position)
{
    for (grid_map::CircleIterator iterator(_grid_map, position, radius); !iterator.isPastEnd(); ++iterator)
    {
        if (_grid_map.at("obstacles", *iterator) >= alpha)
            continue;
        _grid_map.at("obstacles", *iterator) = alpha;
    }
}

geometry_msgs::Pose2D EnviromentCreator::get_leo_pose()
{
    geometry_msgs::Pose2D pose;
    pose.x = _tf.transform.translation.x;  
    pose.y = _tf.transform.translation.y;
    pose.theta = tf2::getYaw(_tf.transform.rotation);
    return pose;
}

grid_map::GridMap *EnviromentCreator::get_grid_map()
{
    return &_grid_map;
}

bool EnviromentCreator::environment_ready()
{
    return _environment_ready;
}

void EnviromentCreator::update_gazebo()
{
    for (unsigned int i = 0; i < _model_states.name.size(); i++)
    {
        if (_model_states.name[i] == "leo")
            continue;
        //else check for collision
        geometry_msgs::Pose leo_pose;
        leo_pose.position.x = _tf.transform.translation.x;
        leo_pose.position.y = _tf.transform.translation.y;
        leo_pose.position.z = _tf.transform.translation.z;
        leo_pose.orientation = _tf.transform.rotation;
        const double COL_RADIUS = 0.5;
        if (sqrt(pow(leo_pose.position.x - _model_states.pose[i].position.x, 2) + pow(leo_pose.position.y - _model_states.pose[i].position.y, 2)) < COL_RADIUS)
        {
            ROS_ERROR("Collision detected with object: %s", _model_states.name[i].c_str());
            std_srvs::Empty srv;
            _pause_gz_client.call(srv);
            ros::shutdown();
        }
        //check for out of bounds
        const double OFFSET = 0.5;
        if (leo_pose.position.x > _map_width / 2 - OFFSET || leo_pose.position.x < -_map_width / 2 + OFFSET || leo_pose.position.y > _map_width / 2 - OFFSET || leo_pose.position.y < -_map_width / 2 + OFFSET)
        {
            ROS_ERROR("Out of bounds detected");
            std_srvs::Empty srv;
            _pause_gz_client.call(srv);
            ros::shutdown();
        }
    }
}