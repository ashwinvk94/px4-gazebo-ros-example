cmake_minimum_required(VERSION 2.8.3)
project(px4-gazebo-ros-example)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(position_setpoint_publisher src/position_setpoint_publisher.cpp)
target_link_libraries(position_setpoint_publisher ${catkin_LIBRARIES})
add_dependencies(position_setpoint_publisher beginner_tutorials_generate_messages_cpp)