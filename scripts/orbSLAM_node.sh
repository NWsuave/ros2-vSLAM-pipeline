#!/usr/bin/env bash
set -e

echo "[orbmono_cpp] Sourcing ROS 2..."
source /opt/ros/humble/setup.bash

echo "[orbmono_cpp] Sourcing ORB-SLAM3 workspace..."
source "$HOME/ws_orbslam3/install/setup.bash"

echo "[orbmono_cpp] Expecting /image_raw/bgr8 from campipe..."

echo "[orbmono_cpp] Starting ORB-SLAM3 mono_node_cpp..."
ros2 run ros2_orb_slam3 mono_node_cpp
