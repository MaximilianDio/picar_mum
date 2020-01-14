cmake_minimum_required(VERSION 2.8.3)
project(trajectory_planner)

find_package(catkin REQUIRED COMPONENTS
  picar_msgs
  rospy
  sensor_msgs
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
