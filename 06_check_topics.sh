#!/usr/bin/env bash
# Read-only topic checks in another terminal. Counts alone do not verify values.
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
ros2 topic info /quadrotor/matlab_motor_commands
ros2 topic info /quadrotor/command/motor_speed
ros2 topic info /quadrotor/odometry
# Uncomment one echo during a run to inspect the message; Ctrl+C cancels waiting.
# ros2 topic echo /quadrotor/command/motor_speed --once
# ros2 topic echo /quadrotor/odometry --once
