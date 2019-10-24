cmake_minimum_required(VERSION 2.8.3)
project(motor)


find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  picar_msgs
  dynamic_reconfigure
  std_srvs
)
generate_dynamic_reconfigure_options(
  cfg/motor.cfg
)
catkin_package()
#catkin_python_setup()

include_directories(${catkin_INCLUDE_DIRS})

