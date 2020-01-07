cmake_minimum_required(VERSION 2.8.3)
project(enc)


find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  picar_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
