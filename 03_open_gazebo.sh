#!/usr/bin/env bash
# Gazebo terminal: models/ must be on the resource path for model://quadrotor.
# No -r: keep Gazebo paused until the MATLAB model is ready.
# The supplied world is the final pitch setup, initial altitude 20 m.
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
export GZ_SIM_RESOURCE_PATH="$HOME/ros2_ws/src/gazebo_matlab_cosim/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim "$HOME/ros2_ws/src/gazebo_matlab_cosim/worlds/empty_world.sdf"
