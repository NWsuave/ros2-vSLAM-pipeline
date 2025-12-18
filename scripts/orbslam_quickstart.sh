#!/usr/bin/env bash
set -euo pipefail

# ====== CONFIG (edit if needed) ======
# ORB-SLAM3 workspace overlay (colcon install space)
WS="${WS:-$HOME/ws_orbslam3}"

# Image topic ORB-SLAM3 expects
IMG_TOPIC="${IMG_TOPIC:-/image_raw/bgr8}"

# Wait settings
TIMEOUT_S="${TIMEOUT_S:-15}"

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

main() {
  echo "=== ORBSLAM QUICKSTART (mono_node_cpp) ==="
  echo "host: $(hostname)"
  echo "date: $(date)"
  echo

  # Source ROS first
  if [ -f /opt/ros/humble/setup.bash ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    set -u
    echo "[env] sourced: /opt/ros/humble/setup.bash"
  else
    echo "[env] ERROR: missing /opt/ros/humble/setup.bash"
    exit 1
  fi

  # Source ORB-SLAM3 overlay
  if [ -f "$WS/install/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$WS/install/setup.bash"
    set -u
    echo "[env] sourced: $WS/install/setup.bash"
  else
    echo "[env] ERROR: missing ORB overlay: $WS/install/setup.bash"
    exit 1
  fi

  echo "[env] ROS_DISTRO=${ROS_DISTRO:-<unset>}"
  echo "[env] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
  echo "[env] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
  echo

  echo "[check] ros2: $(command -v ros2 || echo '<missing>')"
  echo "[check] ORB-SLAM3 pkg:"
  if ! ros2 pkg prefix ros2_orb_slam3 >/dev/null 2>&1; then
    echo "ERROR: ros2_orb_slam3 not found in this environment."
    echo "[debug] try: ros2 pkg list | grep -i orb"
    exit 1
  fi
  echo "OK: ros2_orb_slam3 found"
  echo

  # Wait for camera stream
  wait_topic_advertised "$IMG_TOPIC" "$TIMEOUT_S" || { echo "[debug] camera topics:"; ros2 topic list | grep -E "image_raw|camera_info" || true; exit 1; }
  wait_topic_publishing "$IMG_TOPIC" "$TIMEOUT_S" || exit 1
  echo

  echo "[launch] starting ORB-SLAM3 mono_node_cpp..."
  exec ros2 run ros2_orb_slam3 mono_node_cpp
}

main "$@"

