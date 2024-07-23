#include "enviroment_manager.h"
#include "rover_controller.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leo_autonomy");
    ros::NodeHandle nh;
    RoverController rover_controller(nh);
    EnviromentCreator env_creator(nh, 30);
    ros::Rate rate(10);
    while(ros::ok())
    {
        if(env_creator.environment_ready())
        {
            geometry_msgs::Pose2D current_pose = env_creator.get_leo_pose();
            grid_map::GridMap *grid_map = env_creator.get_grid_map();
            geometry_msgs::Pose2D setpoint;
            setpoint.x = 0.0;
            setpoint.y = 0.0;
            setpoint.theta = 0.0;
            rover_controller.move_to_pose(setpoint, *grid_map, current_pose);
        }
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
