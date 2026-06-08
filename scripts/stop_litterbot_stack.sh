#!/usr/bin/env bash
set -eo pipefail

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

echo "Stopped Gazebo, bridge, and localizer processes."
