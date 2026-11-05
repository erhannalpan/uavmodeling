#!/usr/bin/env bash
# Run from ~/ros2_ws after placing gazebo_matlab_cosim in its src/ directory.
# Build once, and rebuild only when changing the C++ source/build configuration.
source /opt/ros/jazzy/setup.bash
cd "$HOME/ros2_ws" || exit 1
colcon build --packages-select gazebo_matlab_cosim
source "$HOME/ros2_ws/install/setup.bash"
