cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(null_loop)

# Find packages

find_package(gazebo REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Create libraries and executables

add_library(null_loop SHARED null_loop.cc)
target_link_libraries(null_loop ${GAZEBO_LIBRARIES})

add_executable(integrated_main integrated_main.cc)
target_link_libraries(integrated_main ${GAZEBO_LIBRARIES} pthread)


add_executable(independent_listener independent_listener.cc)
target_link_libraries(independent_listener ${GAZEBO_LIBRARIES} pthread)
