#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"
source /opt/ros/humble/setup.bash

WORLD_FILE="${WORLD_FILE:-sim/litterbot_world.sdf}"
LIGHT_SCALE="${LIGHT_SCALE:-1.0}"

if [ "$LIGHT_SCALE" != "1.0" ]; then
  WORLD_FILE="/tmp/litterbot_world_light_${LIGHT_SCALE}.sdf"
  python3 - "$LIGHT_SCALE" sim/litterbot_world.sdf "$WORLD_FILE" <<'PY'
import sys
import xml.etree.ElementTree as ET

scale = float(sys.argv[1])
source = sys.argv[2]
target = sys.argv[3]

tree = ET.parse(source)
root = tree.getroot()
for light in root.findall(".//light"):
    for tag in ("diffuse", "specular"):
        elem = light.find(tag)
        if elem is None or not elem.text:
            continue
        values = [float(v) for v in elem.text.split()]
        for i in range(min(3, len(values))):
            values[i] = max(0.0, min(1.0, values[i] * scale))
        elem.text = " ".join(f"{v:.4f}" for v in values)

tree.write(target, encoding="unicode", xml_declaration=True)
PY
fi

# Software rendering avoids the WSL/OGRE2 GL crash seen on this machine.
LIBGL_ALWAYS_SOFTWARE=1 ign gazebo -r "$WORLD_FILE"
