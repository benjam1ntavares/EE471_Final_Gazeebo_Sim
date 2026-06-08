#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

kill_pidfile() {
  local file="$1"
  local expected="$2"
  if [ -f "$file" ]; then
    local pid
    pid="$(cat "$file" 2>/dev/null || true)"
    if [ -n "$pid" ]; then
      local cmd
      cmd="$(ps -p "$pid" -o args= 2>/dev/null || true)"
      if [ -n "$cmd" ] && [[ "$cmd" == *"$expected"* ]]; then
        kill "$pid" 2>/dev/null || true
      fi
    fi
  fi
}

kill_pidfile /tmp/litterbot_gazebo.pid "scripts/run_gazebo_gui.sh"
kill_pidfile /tmp/litterbot_bridge.pid "scripts/bridge_gazebo_topics.sh"
kill_pidfile /tmp/litterbot_localizer.pid "scripts/run_localizer.sh"
python3 - <<'PY'
import os
import signal

needles = ["ign " + "gazebo", "parameter_" + "bridge", "src/depth_" + "localizer.py"]
self_pid = os.getpid()

for name in os.listdir("/proc"):
    if not name.isdigit():
        continue
    pid = int(name)
    if pid == self_pid:
        continue
    try:
        cmd = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\x00", b" ").decode("utf-8", "ignore")
    except OSError:
        continue
    if any(needle in cmd for needle in needles):
        try:
            os.kill(pid, signal.SIGTERM)
        except OSError:
            pass
PY
sleep 1

source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1

rm -f /tmp/litterbot_gazebo.log /tmp/litterbot_spawn.log /tmp/litterbot_bridge.log /tmp/litterbot_localizer.log
rm -f /tmp/litterbot_gazebo.pid /tmp/litterbot_bridge.pid /tmp/litterbot_localizer.pid

nohup bash scripts/run_gazebo_gui.sh > /tmp/litterbot_gazebo.log 2>&1 &
echo $! > /tmp/litterbot_gazebo.pid
sleep 8

bash scripts/spawn_robot.sh > /tmp/litterbot_spawn.log 2>&1
sleep 3

# Reset ROS2 daemon to prevent DDS discovery failures in WSL2
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start 2>/dev/null || true
sleep 1

nohup bash scripts/bridge_gazebo_topics.sh > /tmp/litterbot_bridge.log 2>&1 &
echo $! > /tmp/litterbot_bridge.pid
sleep 6

echo "Capturing camera calibration..."
bash scripts/run_capture_calibration.sh > /tmp/litterbot_calibration.log 2>&1 || {
  echo "WARNING: camera calibration capture failed; localizer will use CameraInfo fallback."
  cat /tmp/litterbot_calibration.log || true
}

nohup bash scripts/run_localizer.sh > /tmp/litterbot_localizer.log 2>&1 &
echo $! > /tmp/litterbot_localizer.pid
sleep 3

echo "Gazebo PID: $(cat /tmp/litterbot_gazebo.pid)"
echo "Bridge PID: $(cat /tmp/litterbot_bridge.pid)"
echo "Localizer PID: $(cat /tmp/litterbot_localizer.pid)"
echo
echo "--- Gazebo topics ---"
ign topic -l | sort | grep -E "camera|world/litterbot_world/pose|clock|stats" || true
echo
echo "--- Bridge log ---"
tail -20 /tmp/litterbot_bridge.log || true
echo
echo "--- Spawn log ---"
cat /tmp/litterbot_spawn.log || true
echo
echo "--- Calibration log ---"
cat /tmp/litterbot_calibration.log || true
echo
echo "--- Localizer log ---"
tail -40 /tmp/litterbot_localizer.log || true
