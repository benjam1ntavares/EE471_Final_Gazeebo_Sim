#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1

ros2 run ros_gz_sim create \
  -file sim/my_robot.urdf \
  -name simple_bot \
  -x 0 -y 0 -z 0.05
