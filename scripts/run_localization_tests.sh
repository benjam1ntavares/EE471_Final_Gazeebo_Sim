#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

OUT_DIR="${OUT_DIR:-results}"
CSV="${CSV:-$OUT_DIR/localization_trials.csv}"
LOG="${LOG:-/tmp/litterbot_localizer.log}"
SETTLE_SEC="${SETTLE_SEC:-5}"
MODE="${1:-baseline}"
CAPTURE_TRIAL_IMAGES="${CAPTURE_TRIAL_IMAGES:-0}"
CAPTURE_YOLO_IMAGES="${CAPTURE_YOLO_IMAGES:-0}"
PHOTO_DIR="${PHOTO_DIR:-$OUT_DIR/photo_review/$MODE}"

mkdir -p "$OUT_DIR"
if [ "$CAPTURE_TRIAL_IMAGES" = "1" ]; then
  mkdir -p "$PHOTO_DIR"
fi

if [ ! -f "$LOG" ]; then
  echo "Localizer log not found at $LOG"
  echo "Start the stack first with: bash scripts/start_litterbot_stack.sh"
  exit 1
fi

echo "trial,scene,x_m,y_m,z_m,yaw_rad,detection,confidence,pixel_u,pixel_v,depth_surface_m,depth_center_m,estimated_ground_m,estimated_forward_m,estimated_lateral_m,estimated_height_m,ground_truth_m,error_m,error_pct,correction_applied_m,corrected_ground_m,corrected_error_m,corrected_error_pct,raw_image,yolo_image" > "$CSV"

capture_trial_images() {
  local trial="$1"
  local scene="$2"
  local x="$3"
  local y="$4"
  local raw_path=""
  local yolo_path=""

  if [ "$CAPTURE_TRIAL_IMAGES" != "1" ]; then
    echo ","
    return
  fi

  local safe_scene
  safe_scene="$(echo "$scene" | tr -c 'A-Za-z0-9._-' '_')"
  raw_path="$PHOTO_DIR/trial_${trial}_${safe_scene}_x${x}_y${y}_raw.png"
  yolo_path="$PHOTO_DIR/trial_${trial}_${safe_scene}_x${x}_y${y}_yolo.png"

  python3 - "$raw_path" "$yolo_path" "$CAPTURE_YOLO_IMAGES" <<'PY'
import os
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

raw_path, yolo_path, capture_yolo = sys.argv[1:4]

rclpy.init()
node = rclpy.create_node("trial_image_capture")
bridge = CvBridge()
frame = {"value": None}

def cb(msg):
    frame["value"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

sub = node.create_subscription(Image, "/camera/image", cb, 1)
for _ in range(80):
    rclpy.spin_once(node, timeout_sec=0.25)
    if frame["value"] is not None:
        break

if frame["value"] is not None:
    os.makedirs(os.path.dirname(raw_path), exist_ok=True)
    cv2.imwrite(raw_path, frame["value"])
    if capture_yolo == "1":
        from ultralytics import YOLO
        model_path = os.environ.get("YOLO_MODEL", os.path.join(os.getcwd(), "models", "yolo26n.pt"))
        conf = float(os.environ.get("YOLO_CONF", "0.25"))
        target = os.environ.get("YOLO_TARGET", "bottle")
        model = YOLO(model_path)
        class_ids = [idx for idx, name in model.names.items() if name == target]
        kwargs = {"conf": conf, "verbose": False}
        if class_ids:
            kwargs["classes"] = class_ids
        annotated = model(frame["value"], **kwargs)[0].plot()
        cv2.imwrite(yolo_path, annotated)
else:
    raw_path = ""
    yolo_path = ""

node.destroy_node()
rclpy.shutdown()
print(f"{raw_path},{yolo_path if capture_yolo == '1' else ''}")
PY
}

run_trial() {
  local trial="$1"
  local scene="$2"
  local x="$3"
  local y="$4"
  local z="${5:-0.19}"
  local yaw="${6:-0}"
  local log_offset
  log_offset="$(wc -c < "$LOG" 2>/dev/null || echo 0)"

  echo "Trial $trial: scene=$scene x=$x y=$y z=$z yaw=$yaw"
  bash scripts/move_can.sh "$x" "$y" "$z" "$yaw"
  sleep "${POSE_SETTLE_SEC:-1}"
  log_offset="$(wc -c < "$LOG" 2>/dev/null || echo 0)"
  sleep "$SETTLE_SEC"
  local image_paths raw_image yolo_image
  image_paths="$(capture_trial_images "$trial" "$scene" "$x" "$y")"
  raw_image="${image_paths%%,*}"
  yolo_image="${image_paths#*,}"

  python3 - "$LOG" "$CSV" "$trial" "$scene" "$x" "$y" "$z" "$yaw" "$log_offset" "$raw_image" "$yolo_image" <<'PY'
import csv
import math
import re
import sys

log_path, csv_path, trial, scene, x, y, z, yaw, log_offset, raw_image, yolo_image = sys.argv[1:]
log_offset = int(log_offset)

try:
    with open(log_path, "rb") as f:
        f.seek(log_offset)
        text = f.read().decode("utf-8", errors="replace")
except FileNotFoundError:
    text = ""

blocks = re.findall(
    r"=+ DEPTH LOCALIZATION REPORT =+\n(.*?)=+",
    text,
    flags=re.S,
)

row = {
    "trial": trial,
    "scene": scene,
    "x_m": x,
    "y_m": y,
    "z_m": z,
    "yaw_rad": yaw,
    "detection": "",
    "confidence": "",
    "pixel_u": "",
    "pixel_v": "",
    "depth_surface_m": "",
    "depth_center_m": "",
    "estimated_ground_m": "",
    "estimated_forward_m": "",
    "estimated_lateral_m": "",
    "estimated_height_m": "",
    "ground_truth_m": "",
    "error_m": "",
    "error_pct": "",
    "correction_applied_m": "",
    "corrected_ground_m": "",
    "corrected_error_m": "",
    "corrected_error_pct": "",
    "raw_image": raw_image,
    "yolo_image": yolo_image,
}

if blocks:
    block = blocks[-1]

    patterns = {
        "det": r"Detection:\s+(.+?)\s+\(([-+0-9.]+)\)\s+pixel\s+\(([-+0-9]+),\s*([-+0-9]+)\)",
        "surface": r"Depth \(surface\):\s+([-+0-9.]+)\s+m",
        "center": r"Depth \(center\):\s+([-+0-9.]+)\s+m",
        "est": r"Est\. ground distance:\s+([-+0-9.]+)\s+m",
        "fwd": r"Est\. forward offset:\s+([-+0-9.]+)\s+m",
        "lat": r"Est\. lateral offset:\s+([-+0-9.]+)\s+m",
        "height": r"Est\. height offset:\s+([-+0-9.]+)\s+m",
        "gt": r"Ground-truth distance:\s+([-+0-9.]+)\s+m",
        "err": r"Error:\s+([-+0-9.]+)\s+m\s+\(([-+0-9.]+)\s+%\)",
        "corr": r"Correction applied:\s+([-+0-9.]+)\s+m",
        "corr_dist": r"Corrected distance:\s+([-+0-9.]+)\s+m",
        "corr_err": r"Corrected error:\s+([-+0-9.]+)\s+m\s+\(([-+0-9.]+)\s+%\)",
    }

    m = re.search(patterns["det"], block)
    if m:
        row["detection"] = m.group(1).strip()
        row["confidence"] = m.group(2)
        row["pixel_u"] = m.group(3)
        row["pixel_v"] = m.group(4)

    for key, pattern in (
        ("depth_surface_m", patterns["surface"]),
        ("depth_center_m", patterns["center"]),
        ("estimated_ground_m", patterns["est"]),
        ("estimated_forward_m", patterns["fwd"]),
        ("estimated_lateral_m", patterns["lat"]),
        ("estimated_height_m", patterns["height"]),
        ("ground_truth_m", patterns["gt"]),
        ("correction_applied_m", patterns["corr"]),
        ("corrected_ground_m", patterns["corr_dist"]),
    ):
        m = re.search(pattern, block)
        if m:
            row[key] = m.group(1)

    m = re.search(patterns["err"], block)
    if m:
        row["error_m"] = m.group(1)
        row["error_pct"] = m.group(2)

    m = re.search(patterns["corr_err"], block)
    if m:
        row["corrected_error_m"] = m.group(1)
        row["corrected_error_pct"] = m.group(2)

with open(csv_path, "a", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=list(row))
    writer.writerow(row)
PY
}

trial=1

case "$MODE" in
  baseline)
    for x in 1.0 1.5 2.0 2.5 3.0 4.0 5.0 6.0; do
      run_trial "$trial" "distance_sweep" "$x" 0.0
      trial=$((trial + 1))
    done

    for y in -1.0 -0.5 0.0 0.5 1.0; do
      run_trial "$trial" "lateral_sweep" 3.0 "$y"
      trial=$((trial + 1))
    done
    ;;

  extremes)
    while IFS=, read -r x y; do
      run_trial "$trial" "longitudinal_extreme" "$x" "$y"
      trial=$((trial + 1))
    done < <(python3 - <<'PY'
for i in range(100):
    x = 1.0 + i * (6.0 - 1.0) / 99
    print(f"{x:.4f},0.0000")
PY
)

    while IFS=, read -r x y; do
      run_trial "$trial" "lateral_extreme" "$x" "$y"
      trial=$((trial + 1))
    done < <(python3 - <<'PY'
for i in range(70):
    y = -3.5 + i * (3.5 - (-3.5)) / 69
    print(f"3.0000,{y:.4f}")
PY
)
    ;;

  light_20)
    while IFS=, read -r x y; do
      run_trial "$trial" "light_longitudinal" "$x" "$y"
      trial=$((trial + 1))
    done < <(python3 - <<'PY'
for i in range(20):
    x = 1.0 + i * (6.0 - 1.0) / 19
    print(f"{x:.4f},0.0000")
PY
)

    while IFS=, read -r x y; do
      run_trial "$trial" "light_lateral" "$x" "$y"
      trial=$((trial + 1))
    done < <(python3 - <<'PY'
for i in range(20):
    y = -2.75 + i * (2.75 - (-2.75)) / 19
    print(f"3.0000,{y:.4f}")
PY
)
    ;;

  corrected_minimal)
    for x in 1.0 3.25 5.5 7.75 10.0; do
      run_trial "$trial" "longitudinal_corrected" "$x" 0.0
      trial=$((trial + 1))
    done

    for y in -3.5 -1.75 0.0 1.75 3.5; do
      run_trial "$trial" "lateral_corrected" 3.0 "$y"
      trial=$((trial + 1))
    done
    ;;

  clutter_visible)
    # Visibility-controlled clutter test.
    # The nearest central clutter object is at about x=1.85 m, so these target
    # poses keep the bottle closer to the camera than the clutter field.
    for x in 0.9 1.0 1.1 1.2 1.3 1.4; do
      run_trial "$trial" "clutter_visible_center" "$x" 0.0
      trial=$((trial + 1))
    done

    for y in -0.25 0.0 0.25; do
      run_trial "$trial" "clutter_visible_lateral" 1.2 "$y"
      trial=$((trial + 1))
    done
    ;;

  clutter_visible_extended)
    # Extended visibility-controlled clutter test with lateral variation.
    # These 15 additional poses keep the bottle in front of central clutter.
    for x in 0.85 1.00 1.15 1.30 1.45; do
      for y in -0.35 0.00 0.35; do
        run_trial "$trial" "clutter_visible_extended" "$x" "$y"
        trial=$((trial + 1))
      done
    done
    ;;

  clutter_infield_visible)
    # Clutter-field visible test.
    # These positions sit among nearby clutter objects but avoid known direct
    # occluder alignments so the bottle should remain reviewable in-frame.
    while IFS=, read -r x y; do
      run_trial "$trial" "clutter_infield_visible" "$x" "$y"
      trial=$((trial + 1))
    done <<'EOF'
1.75,-0.45
1.90,0.65
2.20,0.95
2.45,-1.05
2.80,0.75
3.20,-0.75
EOF
    ;;

  light_spotcheck)
    # Short lighting spot-check used for extra photo-backed lighting data.
    run_trial "$trial" "light_spotcheck_center_close" 1.20 0.00
    trial=$((trial + 1))
    run_trial "$trial" "light_spotcheck_center_mid" 2.00 0.00
    trial=$((trial + 1))
    run_trial "$trial" "light_spotcheck_lateral" 1.50 0.35
    trial=$((trial + 1))
    ;;

  *)
    echo "Usage: bash scripts/run_localization_tests.sh [baseline|extremes|light_20|corrected_minimal|clutter_visible|clutter_visible_extended|clutter_infield_visible|light_spotcheck]"
    exit 2
    ;;
esac

echo
echo "Wrote CSV: $CSV"
echo "Make graphs with: python3 src/plot_localization_results.py $CSV"
