#!/usr/bin/env bash
set -e

# ==============================================
# ORB-SLAM3 Monocular Camera Driver Runner
#
# What this script does:
# 1) Loads ROS 2 Humble environment
# 2) Loads ORB-SLAM3 workspace overlays
# 3) Runs the monocular camera driver node
#
# Node:
#   ros2_orb_slam3/mono_driver_node.py
# ==============================================

echo "Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

echo "Sourcing ws_tools (image conversion, helpers)..."
if [ -f "$HOME/ws_tools/install/setup.bash" ]; then
    source "$HOME/ws_tools/install/setup.bash"
fi

echo "Sourcing ORB-SLAM3 workspace..."
source "$HOME/ws_orbslam3/install/setup.bash"

echo "Starting ORB-SLAM3 monocular driver node..."
ros2 run ros2_orb_slam3 mono_driver_node.py
