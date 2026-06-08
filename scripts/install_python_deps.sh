#!/usr/bin/env bash
set -eo pipefail

python3 -m pip install --user --upgrade pip
python3 -m pip install --user "numpy<2" "opencv-python<4.10" matplotlib ultralytics

echo
echo "Python dependencies installed."
echo "Restart the stack with: bash scripts/start_litterbot_stack.sh"
