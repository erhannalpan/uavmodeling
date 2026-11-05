#!/usr/bin/env bash
# MATLAB terminal: load ROS and our compiled workspace before starting MATLAB.
# All participating ROS processes use domain 0.
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
/usr/local/MATLAB/R2026a/bin/matlab
