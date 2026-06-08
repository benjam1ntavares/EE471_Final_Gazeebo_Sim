#!/usr/bin/env bash
set -eo pipefail

if [ "$#" -lt 2 ]; then
  echo "Usage: bash scripts/move_can.sh X Y [Z] [YAW]"
  echo "Example: bash scripts/move_can.sh 2.5 0 0.19 0"
  exit 2
fi

x="$1"
y="$2"
z="${3:-0.19}"
yaw="${4:-0}"
qz="$(python3 - <<PY
import math
print(math.sin(float("${yaw}") / 2.0))
PY
)"
qw="$(python3 - <<PY
import math
print(math.cos(float("${yaw}") / 2.0))
PY
)"

# Write target pose to file so the localizer picks it up instantly
# instead of waiting for the slow Gazebo->bridge->ROS pose pipeline
echo "${x} ${y} ${z}" > /tmp/litterbot_target_pose.txt

ign service \
  -s /world/litterbot_world/set_pose \
  --reqtype ignition.msgs.Pose \
  --reptype ignition.msgs.Boolean \
  --timeout 1000 \
  --req "name: 'target_coke_can', position: {x: ${x}, y: ${y}, z: ${z}}, orientation: {x: 0, y: 0, z: ${qz}, w: ${qw}}"
