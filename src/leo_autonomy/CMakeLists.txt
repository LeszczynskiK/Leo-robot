cmake_minimum_required(VERSION 3.0.2)
project(leo_autonomy)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  grid_map_ros
  roscpp
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES leo_autonomy
  CATKIN_DEPENDS gazebo_ros geometry_msgs grid_map_ros roscpp
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include 
)

add_library(obstacle_spawner src/obstacle_spawner.cpp)
target_link_libraries(obstacle_spawner ${catkin_LIBRARIES})
add_library(enviroment_manager src/enviroment_manager.cpp)
target_link_libraries(enviroment_manager obstacle_spawner ${catkin_LIBRARIES})
add_library(rover_controller src/rover_controller.cpp)
target_link_libraries(rover_controller ${catkin_LIBRARIES})

add_executable(leo_autonomy src/leo_autonomy.cpp)
target_link_libraries(leo_autonomy ${catkin_LIBRARIES} enviroment_manager rover_controller)
