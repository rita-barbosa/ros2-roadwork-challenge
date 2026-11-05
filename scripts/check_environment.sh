#!/usr/bin/env bash
set -eu
test "${ROS_DISTRO:-}" = jazzy || { echo 'Source /opt/ros/jazzy/setup.bash first'; exit 1; }
command -v ros2
command -v colcon
command -v g++
echo 'Jazzy environment found; this does not replace build and runtime acceptance.'
