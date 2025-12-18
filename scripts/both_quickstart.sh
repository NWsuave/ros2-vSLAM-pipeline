#!/usr/bin/env bash
set -euo pipefail

# ====== CONFIG (edit if needed) ======
# ORB-SLAM3 workspace overlay (colcon install space)
WS_ORB="${WS_ORB:-$HOME/ws_orbslam3}"

# Optional RTAB-Map overlay (leave empty if using system install)
WS_RTAB="${WS_RTAB:-}"

# Camera topics (adjust to your driver)
RGB_TOPIC="${RGB_TOPIC:-/image_raw}"
CAM_INFO_TOPIC="${CAM_INFO_TOPIC:-/camera_info}"

# ORB-SLAM3 node + expected outputs (adjust if your package uses different names)
ORB_PKG="${ORB_PKG:-ros2_orb_slam3}"
ORB_EXEC="${ORB_EXEC:-mono_node_cpp}"

# What RTAB-Map should use as odom (from ORB-SLAM)
# Common choices: /odom or /orbslam3/odom or /orbslam3/pose (pose needs remap)
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"

# Frames
FRAME_ID="${FRAME_ID:-base_link}"
MAP_FRAME="${MAP_FRAME:-map}"
ODOM_FRAME="${ODOM_FRAME:-odom}"

# Wait settings
TIMEOUT_S="${TIMEOUT_S:-20}"

# Launch RViz automatically? (1=yes, 0=no)
START_RVIZ="${START_RVIZ:-0}"

# RTAB-Map: set to 1 if you have depth, 0 for RGB-only
USE_DEPTH="${USE_DEPTH:-0}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/depth/image_raw}"

# RTAB-Map: set to 1 if you have lidar scan
USE_SCAN="${USE_SCAN:-0}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"

# ====== helpers ======
wait_topic_advertised() {
  local topic="$1"
  local timeout_s="$2"
  local t0
  t0=$(date +%s)

  echo "[wait] checking advertised topic: $topic"
  while true; do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      echo "OK: advertised: $topic"
      return 0
    fi
    if [ $(( $(date +%s) - t0 )) -ge "$timeout_s" ]; then
      echo "ERROR: topic not advertised after ${timeout_s}s: $topic"
      return 1
    fi
    sleep 0.2
  done
}

wait_topic_publishing() {
  local topic="$1"
  local timeout_s="$2"
  local t0
  t0=$(date +%s)

  echo "[wait] checking publishing (1 msg): $topic"
  while true; do
    if timeout 2s ros2 topic echo "$topic" --once >/dev/null 2>&1; then
      echo "OK: $topic is publishing"
      return 0
    fi
    if [ $(( $(date +%s) - t0 )) -ge "$timeout_s" ]; then
      echo "ERROR: $topic not publishing after ${timeout_s}s"
      return 1
    fi
    sleep 0.2
  done
}

wait_tf_frame_exists() {
  local frame="$1"
  local timeout_s="$2"
  local t0
  t0=$(date +%s)

  echo "[wait] checking TF frame exists: $frame"
  while true; do
    # view_frames can be heavy; use tf2_echo quick test
    if timeout 2s ros2 run tf2_ros tf2_echo "$frame" "$frame" >/dev/null 2>&1; then
      echo "OK: TF frame exists: $frame"
      return 0
    fi
    if [ $(( $(date +%s) - t0 )) -ge "$timeout_s" ]; then
      echo "WARN: TF frame not confirmed after ${timeout_s}s: $frame"
      return 1
    fi
    sleep 0.3
  done
}

cleanup() {
  echo
  echo "[shutdown] stopping launched processes..."
  jobs -p | xargs -r kill 2>/dev/null || true
  wait || true
}
trap cleanup EXIT INT TERM

# ====== main ======
main() {
  echo "=== VSLAM BOTH QUICKSTART (ORB-SLAM3 + RTAB-Map) ==="
  echo "host: $(hostname)"
  echo "date: $(date)"
  echo

  set +u
  # Source ROS + overlays
  source /opt/ros/humble/setup.bash
  set -u

  if [ -f "$WS_ORB/install/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$WS_ORB/install/setup.bash"
    set -u
    echo "[env] sourced ORB overlay: $WS_ORB/install/setup.bash"
  else
    echo "[env] ERROR: missing ORB overlay: $WS_ORB/install/setup.bash"
    exit 1
  fi

  if [ -n "${WS_RTAB}" ]; then
    if [ -f "$WS_RTAB/install/setup.bash" ]; then
      set +u
      # shellcheck disable=SC1090
      source "$WS_RTAB/install/setup.bash"
      set -u
      echo "[env] sourced RTAB overlay: $WS_RTAB/install/setup.bash"
    else
      echo "[env] ERROR: WS_RTAB set but missing: $WS_RTAB/install/setup.bash"
      exit 1
    fi
  fi

  echo
  echo "[check] waiting for camera topics..."
  wait_topic_advertised "$RGB_TOPIC" "$TIMEOUT_S"
  wait_topic_advertised "$CAM_INFO_TOPIC" "$TIMEOUT_S"
  wait_topic_publishing "$RGB_TOPIC" "$TIMEOUT_S"
  wait_topic_publishing "$CAM_INFO_TOPIC" "$TIMEOUT_S"

  if [ "$USE_DEPTH" -eq 1 ]; then
    wait_topic_advertised "$DEPTH_TOPIC" "$TIMEOUT_S"
    wait_topic_publishing "$DEPTH_TOPIC" "$TIMEOUT_S"
  fi

  if [ "$USE_SCAN" -eq 1 ]; then
    wait_topic_advertised "$SCAN_TOPIC" "$TIMEOUT_S"
    wait_topic_publishing "$SCAN_TOPIC" "$TIMEOUT_S"
  fi

  echo
  echo "[run] starting ORB-SLAM3..."
  ros2 run "$ORB_PKG" "$ORB_EXEC" \
    --ros-args \
    -p publish_tf:=true \
    -p publish_map_tf:=true \
    -r image:="$RGB_TOPIC" \
    -r camera_info:="$CAM_INFO_TOPIC" \
    >/tmp/orbslam3.log 2>&1 &

  ORB_PID=$!
  echo "OK: ORB-SLAM3 PID=$ORB_PID (log: /tmp/orbslam3.log)"

  echo
  echo "[check] waiting for odom topic (from ORB-SLAM3): $ODOM_TOPIC"
  wait_topic_advertised "$ODOM_TOPIC" "$TIMEOUT_S" || {
    echo "ERROR: expected odom topic not advertised: $ODOM_TOPIC"
    echo "HINT: set ODOM_TOPIC to your ORB output (e.g. /orbslam3/odom)."
    exit 1
  }
  wait_topic_publishing "$ODOM_TOPIC" "$TIMEOUT_S" || {
    echo "ERROR: expected odom topic not publishing: $ODOM_TOPIC"
    exit 1
  }

  # Best-effort TF frame checks (won't hard-fail if tf2_echo check is flaky)
  wait_tf_frame_exists "$MAP_FRAME" 5 || true
  wait_tf_frame_exists "$ODOM_FRAME" 5 || true

  echo
  echo "[run] starting RTAB-Map (external odom)..."
  RTAB_ARGS=(
    "use_sim_time:=false"
    "frame_id:=$FRAME_ID"
    "odom_topic:=$ODOM_TOPIC"
    "rgb_topic:=$RGB_TOPIC"
    "camera_info_topic:=$CAM_INFO_TOPIC"
    "approx_sync:=true"
    "queue_size:=30"
    "sync_queue_size:=30"
  )

  if [ "$USE_DEPTH" -eq 1 ]; then
    RTAB_ARGS+=("subscribe_depth:=true" "depth_topic:=$DEPTH_TOPIC")
  else
    RTAB_ARGS+=("subscribe_depth:=false")
  fi

  if [ "$USE_SCAN" -eq 1 ]; then
    RTAB_ARGS+=("subscribe_scan:=true" "scan_topic:=$SCAN_TOPIC")
  else
    RTAB_ARGS+=("subscribe_scan:=false")
  fi

  # NOTE: Using rtabmap.launch.py keeps it simple + standard topics (/rtabmap/*)
  ros2 launch rtabmap_ros rtabmap.launch.py "${RTAB_ARGS[@]}" \
    >/tmp/rtabmap.log 2>&1 &

  RTAB_PID=$!
  echo "OK: RTAB-Map PID=$RTAB_PID (log: /tmp/rtabmap.log)"

  if [ "$START_RVIZ" -eq 1 ]; then
    echo
    echo "[run] starting RViz2..."
    rviz2 >/tmp/rviz2.log 2>&1 &
    RVIZ_PID=$!
    echo "OK: RViz2 PID=$RVIZ_PID (log: /tmp/rviz2.log)"
  fi

  echo
  echo "=== RUNNING ==="
  echo "Camera:    $RGB_TOPIC  |  $CAM_INFO_TOPIC"
  if [ "$USE_DEPTH" -eq 1 ]; then echo "Depth:     $DEPTH_TOPIC"; fi
  if [ "$USE_SCAN" -eq 1 ]; then echo "Scan:      $SCAN_TOPIC"; fi
  echo "ORB Odom:  $ODOM_TOPIC"
  echo "RTAB:      /rtabmap/* topics"
  echo
  echo "Tips:"
  echo "  - Watch logs: tail -f /tmp/orbslam3.log  /tmp/rtabmap.log"
  echo "  - Save map DB: ros2 service call /rtabmap/save_map rtabmap_msgs/srv/SaveMap \"{database_path: '~/rtabmap.db'}\""
  echo
  echo "Press Ctrl+C to stop."
  echo

  # Keep script alive while child jobs run
  wait
}

main "$@"
