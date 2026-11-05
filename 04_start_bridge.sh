#!/usr/bin/env bash
# Bridge terminal: translate the same two message mappings used in the project.
# Commands travel to Gazebo; odometry returns to ROS. The historical @ syntax
# configures bidirectional mappings, so publisher counts may include the bridge.
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
ros2 run ros_gz_bridge parameter_bridge \
  '/quadrotor/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators' \
  '/quadrotor/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
