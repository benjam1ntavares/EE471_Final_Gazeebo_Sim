#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1

export YOLO_MODEL="${YOLO_MODEL:-models/yolo26n.pt}"
export YOLO_TARGET="${YOLO_TARGET:-bottle}"
export YOLO_SHOW="${YOLO_SHOW:-1}"

python3 src/yolo_live_viewer.py
