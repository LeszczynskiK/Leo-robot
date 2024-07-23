#!/bin/bash
ROOT_DIR=$(pwd)
cd ~/RoverAutonomy
source /opt/ros/noetic/setup.bash
catkin build -DCMAKE_EXPORT_COMPILE_COMMANDS=1
source ~/RoverAutonomy/devel/setup.bash
cd $ROOT_DIR