#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1

export YOLO_MODEL="${YOLO_MODEL:-models/yolo26n.pt}"
export YOLO_TARGET="${YOLO_TARGET:-bottle}"
export ENABLE_ERROR_CORRECTION="${ENABLE_ERROR_CORRECTION:-0}"

python3 - <<'PY'
import importlib.util
import sys

missing = [
    pkg for pkg in ("ultralytics", "cv2", "numpy")
    if importlib.util.find_spec(pkg) is None
]

if missing:
    print("Missing Python packages: " + ", ".join(missing), file=sys.stderr)
    print("Install them with: bash scripts/install_python_deps.sh", file=sys.stderr)
    sys.exit(1)
PY

python3 src/depth_localizer.py
