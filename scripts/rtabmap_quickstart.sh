#!/usr/bin/env bash
set -euo pipefail

# ====== CONFIG (edit if needed) ======
# Workspace/overlay to source (set to ws_tools if you want campipe tools in the same env)
WS="${WS:-/opt/ros/humble}"          # fallback, but we still source /opt/ros/humble below
OVERLAY="${OVERLAY:-$HOME/ws_tools}" # optional overlay to source if it exists

# Expected topics for RTAB-Map RGB+Scan (no depth)
RGB_TOPIC="${RGB_TOPIC:-/image_raw/bgr8}"
CAMINFO_TOPIC="${CAMINFO_TOPIC:-/camera_info}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"

# Frames
FRAME_ID="${FRAME_ID:-base_link}"
ODOM_FRAME_ID="${ODOM_FRAME_ID:-odom}"
MAP_FRAME_ID="${MAP_FRAME_ID:-map}"

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
  echo "=== RTABMAP QUICKSTART (RGB + Scan, NO depth) ==="
  echo "host: $(hostname)"
  echo "date: $(date)"
  echo

  # Always source ROS first (so ros2 exists)
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

  # Optional overlay (ws_tools) — helps if rtabmap is in your overlay or you want consistent env
  if [ -f "$OVERLAY/install/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1090
    source "$OVERLAY/install/setup.bash"
    set -u
    echo "[env] sourced: $OVERLAY/install/setup.bash"
  else
    echo "[env] note: no overlay sourced (missing: $OVERLAY/install/setup.bash)"
  fi

  echo "[env] ROS_DISTRO=${ROS_DISTRO:-<unset>}"
  echo "[env] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
  echo "[env] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
  echo

  echo "[check] ros2: $(command -v ros2 || echo '<missing>')"
  echo "[check] RTAB-Map pkgs:"
  if ! ros2 pkg prefix rtabmap_slam >/dev/null 2>&1; then
    echo "ERROR: rtabmap_slam not found in this environment."
    echo "[debug] try: ros2 pkg list | grep -i rtab"
    exit 1
  fi
  echo "OK: rtabmap_slam found"
  echo

  # Wait for topics to exist first
  wait_topic_advertised "$RGB_TOPIC" "$TIMEOUT_S" || { echo "[debug] topics:"; ros2 topic list | head -n 200; exit 1; }
  wait_topic_advertised "$CAMINFO_TOPIC" "$TIMEOUT_S" || { echo "[debug] topics:"; ros2 topic list | head -n 200; exit 1; }
  wait_topic_advertised "$SCAN_TOPIC" "$TIMEOUT_S" || { echo "[debug] topics:"; ros2 topic list | head -n 200; exit 1; }

  echo
  # Then confirm they are actually publishing
  wait_topic_publishing "$RGB_TOPIC" "$TIMEOUT_S" || exit 1
  wait_topic_publishing "$CAMINFO_TOPIC" "$TIMEOUT_S" || exit 1
  wait_topic_publishing "$SCAN_TOPIC" "$TIMEOUT_S" || exit 1

  echo
  echo "[launch] starting RTAB-Map (publish map->odom TF, no depth)..."
  exec ros2 run rtabmap_slam rtabmap --ros-args \
    -p frame_id:="$FRAME_ID" \
    -p odom_frame_id:="$ODOM_FRAME_ID" \
    -p map_frame_id:="$MAP_FRAME_ID" \
    -p publish_tf:=true \
    -p subscribe_depth:=false \
    -p subscribe_rgb:=true \
    -p subscribe_scan:=true \
    -p approx_sync:=true \
    -r rgb/image:="$RGB_TOPIC" \
    -r rgb/camera_info:="$CAMINFO_TOPIC" \
    -r scan:="$SCAN_TOPIC"
}

main "$@"
