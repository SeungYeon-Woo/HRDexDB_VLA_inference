#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

echo "[1/3] xArm joint state"
ros2 topic echo /right/xarm/joint_states --once

echo "[2/3] xArm robot state / EEF pose"
ros2 topic echo /right/xarm/robot_states --once

echo "[3/3] Inspire direct state"
"${PYTHON:-python}" check_inspire_state.py --hz 1 --samples 3
