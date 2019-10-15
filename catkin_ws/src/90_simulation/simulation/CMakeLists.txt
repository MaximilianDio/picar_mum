cmake_minimum_required(VERSION 2.8.3)
project(simulation)


find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  picar_msgs
  gazebo_ros
  gazebo_plugins
  roscpp
)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})


include_directories(include ${catkin_INCLUDE_DIRS}
                            ${GAZEBO_INCLUDE_DIRS}
                            ${Boost_INCLUDE_DIR})
                            
                            
add_library(picarPlugin plugins/picarPlugin.cpp)
add_dependencies(picarPlugin ${catkin_EXPORTED_TARGETS})
target_link_libraries(picarPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)
