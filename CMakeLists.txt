# CMakeLists.txt
# Original working build configuration supplied by Erhan; comments expanded.
# Active CMake commands and their arguments are unchanged.
#
# Place this file at the ROS package root, beside package.xml.
# The adapter source must be at src/motor_command_adapter.cpp relative to it.
# colcon invokes this configuration when building gazebo_matlab_cosim.

# Require CMake 3.8 or newer to configure this package.
cmake_minimum_required(VERSION 3.8)

# Set the package/project name. It should match <name> in package.xml.
# PROJECT_NAME is later used to choose the executable's installation directory.
project(gazebo_matlab_cosim)

# Enable useful compiler diagnostics for GNU and Clang C++ compilers.
# -Wall and -Wextra enable warning groups; -Wpedantic reports language-standard
# conformance issues. These flags do not implement runtime command validation.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Locate the build system and the C++/message dependencies.
# REQUIRED stops configuration if a dependency cannot be found.
# Source the ROS installation and any needed workspace overlay before building.
find_package(ament_cmake REQUIRED)    # ROS 2 CMake package/build support.
find_package(rclcpp REQUIRED)         # C++ node, publisher, subscriber, executor.
find_package(std_msgs REQUIRED)       # Incoming Float64MultiArray message.
find_package(actuator_msgs REQUIRED)  # Outgoing Actuators motor-command message.

# Compile the adapter source into an executable named motor_command_adapter.
# The executable name differs from the package name; both are used by ros2 run.
add_executable(motor_command_adapter
  src/motor_command_adapter.cpp
)

# Attach the ROS dependencies to this executable, supplying their include paths
# and link requirements. find_package locates a dependency; this command makes
# it available to the target being built.
ament_target_dependencies(motor_command_adapter
  rclcpp
  std_msgs
  actuator_msgs
)

# Install the executable where ROS 2 expects package executables:
# <install-prefix>/lib/gazebo_matlab_cosim/motor_command_adapter.
# After building and sourcing install/setup.bash, launch it with:
#   ros2 run gazebo_matlab_cosim motor_command_adapter
# This original rule installs only the executable. It does not install models/
# or worlds/; our historical Gazebo commands reference those source directories.
install(TARGETS motor_command_adapter
  DESTINATION lib/${PROJECT_NAME}
)

# Add matching find_package/target dependencies here if the source later starts
# using other packages. package.xml must also declare the relevant dependencies.
# find_package(<dependency> REQUIRED)

# Optional development-time lint setup. BUILD_TESTING is a CMake configuration
# option; this block does not run flight simulations or validate physical models.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Preserve the original template settings: the copyright check and cpplint
  # are skipped by marking their checks as already found. These settings do not
  # grant a license or establish that the skipped checks have actually passed.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  # Discover the remaining applicable lint tests from package.xml test entries.
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize the ament package and generate package metadata/environment support
# so an installed, sourced workspace can discover gazebo_matlab_cosim.
ament_package()
