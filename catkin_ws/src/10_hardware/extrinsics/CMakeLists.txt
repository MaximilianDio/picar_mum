cmake_minimum_required(VERSION 2.8.3)
project(extrinsics)


find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  picar_msgs
  ground_projection
)

catkin_package()
catkin_python_setup()

include_directories(${catkin_INCLUDE_DIRS})

