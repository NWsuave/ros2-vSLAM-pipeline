#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-rtab}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RVIZ_DIR="$ROOT/rviz"

case "$MODE" in
  rtab) CFG="$RVIZ_DIR/rtab.rviz" ;;
  orb)  CFG="$RVIZ_DIR/orb.rviz" ;;
  both) CFG="$RVIZ_DIR/both.rviz" ;;
  *)
    echo "Usage: $0 {rtab|orb|both}"
    exit 1
    ;;
esac

echo "[rviz_profile] launching RViz profile: $MODE"
exec rviz2 -d "$CFG"
