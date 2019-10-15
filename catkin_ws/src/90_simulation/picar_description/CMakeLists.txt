cmake_minimum_required(VERSION 2.8.3)
project(picar_description)


find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  picar_msgs
  gazebo_ros
  gazebo_plugins
  roscpp
)

find_package(gazebo REQUIRED)


catkin_package(
)


include_directories(include ${catkin_INCLUDE_DIRS}
                            ${GAZEBO_INCLUDE_DIRS})
