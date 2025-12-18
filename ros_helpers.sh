 # --- ROS / SLAM shortcuts ---
campipe() {
  trap 'echo "[campipe] Interrupted. Exiting."; return 130' INT

  echo "[campipe] Sourcing ROS 2..."
  source /opt/ros/humble/setup.bash

  echo "[campipe] Sourcing ws_tools workspace..."
  source "$HOME/ws_tools/install/setup.bash"

  echo "[campipe] Waiting for /image_raw from robot... (Ctrl-C to stop)"
  until ros2 topic echo /image_raw --once >/dev/null 2>&1; do
    sleep 0.2
  done

  echo "[campipe] /image_raw detected."
  echo "[campipe] Starting Video Conversion and Publishing Transforms..."
  ros2 launch "$HOME/ws_tools/launch/campipe.launch.py"
}

orbmono_driver() {
  trap 'echo "[orbmono_driver] Interrupted. Exiting."; return 130' INT

  echo "[orbmono_driver] Sourcing ROS 2..."
  source /opt/ros/humble/setup.bash

  echo "[orbmono_driver] Sourcing ORB-SLAM3 workspace..."
  source "$HOME/ws_orbslam3/install/setup.bash"

  echo "[orbmono_driver] Starting ORB-SLAM3 mono_driver_node.py..."
  ros2 run ros2_orb_slam3 mono_driver_node.py
}

orbmono_node() {
trap 'echo "[orbmono_node] Interrupted. Exiting."; return 130' INT

  echo "[orbmono_node] Sourcing ROS 2..."
  source /opt/ros/humble/setup.bash

  echo "[orbmono_node] Sourcing ORB-SLAM3 workspace..."
  source "$HOME/ws_orbslam3/install/setup.bash"

  echo "[orbmono_node] Waiting for /image_raw/bgr8... (run campipe first) (Ctrl-C to stop)"
  until ros2 topic echo /image_raw/bgr8 --once >/dev/null 2>&1; do
    sleep 0.2
  done

  echo "[orbmono_node] /image_raw/bgr8 detected."
  echo "[orbmono_node] Starting ORB-SLAM3 mono_node_cpp..."
  ros2 run ros2_orb_slam3 mono_node_cpp
}

roscheck() {
  echo "=== ROSCHECK ==="
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
  echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
  echo

  echo "[topics] camera + tf:"
  ros2 topic list | egrep -n "(/image_raw$|/image_raw/bgr8$|/camera_info$|/tf$|/tf_static$|/scan$|/od>
  echo

  echo "[hz] (will run quickly if topics exist)"
  ros2 topic hz /image_raw --window 10 2>/dev/null | head -n 5 || echo "no /image_raw"
  ros2 topic hz /image_raw/bgr8 --window 10 2>/dev/null | head -n 5 || echo "no /image_raw/bgr8"
  ros2 topic hz /scan --window 10 2>/dev/null | head -n 5 || echo "no /scan"
}

tfcheck() {
  echo "=== TFCHECK ==="
  echo "[1] base_link -> camera_link"
  ros2 run tf2_ros tf2_echo base_link camera_link --once 2>/dev/null || echo "MISSING: base_link->cam>
  echo
  echo "[2] base_link -> base_scan"
  ros2 run tf2_ros tf2_echo base_link base_scan --once 2>/dev/null || echo "MISSING: base_link->base_>
}

framespdf() {
  echo "[framespdf] Generating TF frame graph..."
  ros2 run tf2_tools view_frames
  echo "[framespdf] Output saved as frames.pdf"
}
