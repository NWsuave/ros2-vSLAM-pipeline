# ~/ros_helpers.sh
# --- ROS / SLAM shortcuts (WSL/Ubuntu, ROS 2 Humble) ---

# Source ROS and (optionally) overlays.
rosenv() {
  # Usage:
  #   rosenv              -> source /opt/ros/humble
  #   rosenv tools        -> source ROS + ws_tools
  #   rosenv orb          -> source ROS + ws_orbslam3
  #   rosenv all          -> source ROS + ws_tools + ws_orbslam3
  source /opt/ros/humble/setup.bash

  case "${1:-}" in
    tools)
      [ -f "$HOME/ws_tools/install/setup.bash" ] && source "$HOME/ws_tools/install/setup.bash"
      ;;
    orb)
      [ -f "$HOME/ws_orbslam3/install/setup.bash" ] && source "$HOME/ws_orbslam3/install/setup.bash"
      ;;
    all)
      [ -f "$HOME/ws_tools/install/setup.bash" ] && source "$HOME/ws_tools/install/setup.bash"
      [ -f "$HOME/ws_orbslam3/install/setup.bash" ] && source "$HOME/ws_orbslam3/install/setup.bash"
      ;;
  esac
}

# Safer wait: checks topic exists in list, and optionally waits for a message with timeout.
wait_topic() {
  # Usage: wait_topic /topic_name [seconds]
  local T="${1:?topic required}"
  local S="${2:-10}"

  echo "[wait_topic] waiting up to ${S}s for $T to be advertised..."
  local start end
  start=$(date +%s)
  end=$((start + S))

  while [ "$(date +%s)" -lt "$end" ]; do
    if ros2 topic list 2>/dev/null | grep -qx "$T"; then
      echo "[wait_topic] advertised: $T"
      return 0
    fi
    sleep 0.2
  done

  echo "[wait_topic] TIMEOUT: $T not advertised after ${S}s"
  return 1
}

ros_health() {
  echo "=== ROS HEALTH ==="
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
  echo
  ros2 topic list | egrep "(/image_raw$|/image_raw/bgr8$|/camera_info$|/scan$|/odom$|/map$|/rtabmap/cloud_map$|/tf$|/tf_static$)" || true
}

tf_health() {
  echo "=== TF HEALTH ==="
  timeout 2s ros2 run tf2_ros tf2_echo odom base_link --once >/dev/null && echo "OK odom->base_link" || echo "NO odom->base_link"
  timeout 2s ros2 run tf2_ros tf2_echo base_link camera_link --once >/dev/null && echo "OK base_link->camera_link" || echo "NO base_link->camera_link"
  timeout 2s ros2 run tf2_ros tf2_echo map odom --once >/dev/null && echo "OK map->odom" || echo "NO map->odom (start RTAB-Map)"
}

campipe() {
  trap 'echo "[campipe] Interrupted."; return 130' INT
  rosenv tools

  # wait for robot camera raw stream to exist
  wait_topic /image_raw 10 || return 1

  echo "[campipe] launching campipe..."
  ros2 launch "$HOME/ws_tools/launch/campipe.launch.py"
}

# RTAB-Map RGB + Scan (NO depth) — your working command
rtabmap_rgbscan() {
  trap 'echo "[rtabmap_rgbscan] Interrupted."; return 130' INT
  rosenv

  echo "[rtabmap_rgbscan] expects: /image_raw/bgr8  /camera_info  /scan"
  wait_topic /image_raw/bgr8 10 || return 1
  wait_topic /camera_info 10 || return 1
  wait_topic /scan 10 || return 1

  ros2 run rtabmap_slam rtabmap --ros-args \
    -p frame_id:=base_link \
    -p odom_frame_id:=odom \
    -p map_frame_id:=map \
    -p publish_tf:=true \
    -p subscribe_depth:=false \
    -p subscribe_rgb:=true \
    -p subscribe_scan:=true \
    -p approx_sync:=true \
    -r rgb/image:=/image_raw/bgr8 \
    -r rgb/camera_info:=/camera_info \
    -r scan:=/scan
}

orbmono_node() {
  trap 'echo "[orbmono_node] Interrupted."; return 130' INT
  rosenv orb

  # ORB-SLAM needs the bgr8 image stream
  wait_topic /image_raw/bgr8 10 || return 1

  echo "[orbmono_node] starting ORB-SLAM3 mono_node_cpp..."
  ros2 run ros2_orb_slam3 mono_node_cpp
}

orbmono_driver() {
  trap 'echo "[orbmono_driver] Interrupted."; return 130' INT
  rosenv orb

  echo "[orbmono_driver] starting mono_driver_node.py..."
  ros2 run ros2_orb_slam3 mono_driver_node.py
}

# RViz profiles (adjust repo path if needed)
rviz_rtab() { rviz2 -d "$HOME/monoOrbSLAM/rviz/rtab.rviz"; }
rviz_orb()  { rviz2 -d "$HOME/monoOrbSLAM/rviz/orb.rviz"; }
rviz_both() { rviz2 -d "$HOME/monoOrbSLAM/rviz/both.rviz"; }

# Optional: reset RTAB-Map db (fresh demo)
rtabmap_reset() {
  rm -f "$HOME/.ros/rtabmap.db"
  echo "[rtabmap_reset] cleared ~/.ros/rtabmap.db"
}

ros_help() {
  cat <<'EOF'
=== ROS Helper Commands ===

Environment
  rosenv [tools|orb|all]     Source ROS 2 and optional workspace overlays

Camera / Video
  campipe                   Start camera pipeline (YUY2 → BGR8 + camera TF)

ORB-SLAM3
  orbmono_node              Run ORB-SLAM3 monocular SLAM node
  orbmono_driver            Run ORB-SLAM3 Python driver (restartable)

RTAB-Map
  rtabmap_rgbscan           Run RTAB-Map (RGB camera + LiDAR, no depth)
  rtabmap_reset             Clear RTAB-Map database (fresh mapping run)

Visualization
  rviz_rtab                 RViz: RTAB-Map (2D map + 3D cloud + camera)
  rviz_orb                  RViz: ORB-SLAM (camera + TF)
  rviz_both                 RViz: RTAB-Map + camera + TF

Diagnostics
  ros_health                Check key ROS topics are alive
  tf_health                 Check critical TF links
  wait_topic <topic> [sec]  Wait for a topic to appear (with timeout)

EOF
}
