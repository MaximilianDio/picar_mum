cmake_minimum_required(VERSION 2.8.3)
project(picar_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  LanePose.msg
  CarCmd.msg
  Pose2DStamped.msg
  WheelSpeedStamped.msg
)

add_service_files(
  FILES
  SetValue.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
)


include_directories(${catkin_INCLUDE_DIRS})
