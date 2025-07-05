#!/bin/bash
cd ~/patrol_robot_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch patrol_system patrol_launch.py