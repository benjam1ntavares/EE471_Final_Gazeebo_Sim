#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1

echo "--- Processes ---"
ps -ef | grep -E "ign gazebo|parameter_bridge|depth_localizer" | grep -v grep || true

echo
echo "--- Gazebo topics ---"
timeout 5s ign topic -l | sort | grep -E "camera|world/litterbot_world/pose|clock|stats" || true

echo
echo "--- ROS topics ---"
timeout 8s ros2 topic list --no-daemon --spin-time 2 | sort | grep -E "camera|world/litterbot_world/pose|clock" || true

echo
echo "--- ROS nodes ---"
timeout 8s ros2 node list --no-daemon --spin-time 2 || true

echo
echo "--- Logs ---"
echo "Gazebo:    /tmp/litterbot_gazebo.log"
echo "Spawn:     /tmp/litterbot_spawn.log"
echo "Bridge:    /tmp/litterbot_bridge.log"
echo "Localizer: /tmp/litterbot_localizer.log"
