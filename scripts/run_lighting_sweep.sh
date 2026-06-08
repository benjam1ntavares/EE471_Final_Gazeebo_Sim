#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

OUT_DIR="${OUT_DIR:-results}"
FINAL_CSV="${CSV:-$OUT_DIR/lighting_sweep_trials.csv}"
SETTLE_SEC="${SETTLE_SEC:-2}"
POSE_SETTLE_SEC="${POSE_SETTLE_SEC:-1}"
YOLO_TARGET="${YOLO_TARGET:-bottle}"
YOLO_CONF="${YOLO_CONF:-0.15}"
WORLD_FILE="${WORLD_FILE:-sim/litterbot_world.sdf}"

mkdir -p "$OUT_DIR"
rm -f "$FINAL_CSV"

for percent in 100 90 80 70 60 50 40 30 20 10; do
  scale="$(printf '0.%02d' "$percent")"
  if [ "$percent" = "100" ]; then
    scale="1.00"
  fi
  temp_csv="$OUT_DIR/lighting_${percent}.csv"

  echo
  echo "===== Lighting test: ${percent}% brightness ====="
  bash scripts/stop_litterbot_stack.sh || true
  WORLD_FILE="$WORLD_FILE" LIGHT_SCALE="$scale" YOLO_TARGET="$YOLO_TARGET" YOLO_CONF="$YOLO_CONF" bash scripts/start_litterbot_stack.sh
  SETTLE_SEC="$SETTLE_SEC" POSE_SETTLE_SEC="$POSE_SETTLE_SEC" \
    CSV="$temp_csv" YOLO_TARGET="$YOLO_TARGET" YOLO_CONF="$YOLO_CONF" \
    CAPTURE_TRIAL_IMAGES="${CAPTURE_TRIAL_IMAGES:-0}" \
    CAPTURE_YOLO_IMAGES="${CAPTURE_YOLO_IMAGES:-0}" \
    PHOTO_DIR="${PHOTO_DIR:-$OUT_DIR/photo_review/lighting}/light_${percent}" \
    bash scripts/run_localization_tests.sh light_20

  python3 - "$temp_csv" "$FINAL_CSV" "$percent" "$scale" <<'PY'
import csv
import os
import sys

src, dst, percent, scale = sys.argv[1:]

with open(src, newline="", encoding="utf-8") as f:
    rows = list(csv.DictReader(f))

if not rows:
    raise SystemExit(f"No rows found in {src}")

fieldnames = ["light_percent", "light_scale"] + list(rows[0].keys())
write_header = not os.path.exists(dst)

with open(dst, "a", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(f, fieldnames=fieldnames)
    if write_header:
        writer.writeheader()
    for row in rows:
        out = {"light_percent": percent, "light_scale": scale}
        out.update(row)
        writer.writerow(out)
PY
done

bash scripts/stop_litterbot_stack.sh || true

echo
echo "Wrote lighting sweep CSV: $FINAL_CSV"
echo "Plot with: python3 src/plot_lighting_results.py $FINAL_CSV"
