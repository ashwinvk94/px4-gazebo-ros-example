# PX4 Gazebo Examples

This repository contains examples of how to send position/velocity/acceleration/yaw/yawrate setpoints to a px4 based system. This examples are based the gazebo simulator
and uses mavros to communicate with the px4 system.

[![IMAGE ALT TEXT HERE](https://github.com/ashwinvk94/px4-gazebo-ros-example/blob/master/demo.png?raw=true)](https://youtu.be/nsGc7zbpC5U)

## How to building and run the code

run the following commands:
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursivebash ubuntu.sh
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
sudo reboot
wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
bash ubuntu_sim_ros_melodic.sh
cd
mkdir -p catkin_ws/src
cd catkin_ws/src
https://github.com/ashwinvk94/px4-gazebo-ros-example
https://github.com/lrse/ros-keyboard
cd ~/catkin_ws
catkin_make
source /devel/setup.bash
roscd keyboard
roscd px4-gazebo-ros-example
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4-gazebo-ros-example inspection.launch # automated inspection demo
roslaunch px4-gazebo-ros-example manual.launch # manual flight using keyboard. Find instructions below
```
### Manual Flight Keyboard Instructions
**arrow** keys => change position setpoint with respect to the world fixed frame  
**w/s** keys => change height setpoints up/down  
**a/d** keys => change yaw setpoints by +/- 45 deg  
**l** key => land/quit the program  
