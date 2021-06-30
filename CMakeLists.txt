cmake_minimum_required(VERSION 2.8.3)
project(px4-gazebo-ros-example)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy keyboard std_msgs genmsg)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(position_setpoint_publisher src/position_setpoint_publisher.cpp)
target_link_libraries(position_setpoint_publisher ${catkin_LIBRARIES})
add_dependencies(position_setpoint_publisher px4-gazebo-ros-example_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(manual_flight src/manual_flight.cpp)
target_link_libraries(manual_flight ${catkin_LIBRARIES})
add_dependencies(manual_flight px4-gazebo-ros-example_generate_messages_cpp ${catkin_EXPORTED_TARGETS})