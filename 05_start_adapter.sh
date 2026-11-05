#!/usr/bin/env bash
# Adapter terminal: Float64MultiArray.data -> Actuators.velocity, four rad/s refs.
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
ros2 run gazebo_matlab_cosim motor_command_adapter
