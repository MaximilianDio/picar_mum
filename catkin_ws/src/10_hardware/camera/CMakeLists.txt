cmake_minimum_required(VERSION 2.8.3)
project(camera)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  picar_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
    cfg/cam_params.cfg
)


catkin_package()


include_directories(${catkin_INCLUDE_DIRS})
