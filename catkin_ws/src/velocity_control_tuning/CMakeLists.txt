cmake_minimum_required(VERSION 2.8.3)
project(velocity_control_tuning)

find_package(catkin REQUIRED COMPONENTS
  picar_msgs
  rospy
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
