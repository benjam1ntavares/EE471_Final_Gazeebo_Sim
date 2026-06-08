#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1

python3 src/capture_camera_calibration.py
