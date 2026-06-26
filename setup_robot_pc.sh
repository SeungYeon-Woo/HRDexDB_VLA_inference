#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$HOME/VCL/VLA_vanilla_test}"
VLA_REPO_URL="${VLA_REPO_URL:-https://github.com/SeungYeon-Woo/HRDexDB_VLA_inference.git}"
PARADEX_REPO_URL="${PARADEX_REPO_URL:-https://github.com/willi19/paradex.git}"
PARADEX_BRANCH="${PARADEX_BRANCH:-xarm-f1}"

mkdir -p "$WORKSPACE_ROOT"
cd "$WORKSPACE_ROOT"

clone_or_update() {
  local url="$1"
  local dir="$2"
  local branch="${3:-}"

  if [ -d "$dir/.git" ]; then
    echo "[skip] $dir already exists"
    return 0
  fi

  if [ -n "$branch" ]; then
    git clone --branch "$branch" --single-branch "$url" "$dir"
  else
    git clone "$url" "$dir"
  fi
}

clone_or_update "$VLA_REPO_URL" vla_inference
clone_or_update "$PARADEX_REPO_URL" paradex "$PARADEX_BRANCH"

cd "$WORKSPACE_ROOT/vla_inference"
git submodule update --init --recursive
cd "$WORKSPACE_ROOT"


# Install the HRDexDB 480x640 camera stream script into paradex.
if [ -f "$WORKSPACE_ROOT/vla_inference/camera/stream_client_hrdex.py" ]; then
  mkdir -p "$WORKSPACE_ROOT/paradex/src/capture/camera"
  cp "$WORKSPACE_ROOT/vla_inference/camera/stream_client_hrdex.py"      "$WORKSPACE_ROOT/paradex/src/capture/camera/stream_client_hrdex.py"
  chmod +x "$WORKSPACE_ROOT/paradex/src/capture/camera/stream_client_hrdex.py"
  echo "[ok] installed paradex/src/capture/camera/stream_client_hrdex.py"
fi

cd "$WORKSPACE_ROOT/vla_inference"
if [ -f requirements.txt ]; then
  python3 -m pip install -r requirements.txt
fi

cat <<EOF

Robot PC workspace is ready at:
  $WORKSPACE_ROOT

Next terminals:

1. RLDX server terminal:
   cd $WORKSPACE_ROOT/vla_inference
   RLDX_ROOT=$WORKSPACE_ROOT/vla_inference/third_party/RLDX-1 \\
   RLDX_MODEL_PATH=/research/ckpt/fk_lora_r16_b16_20k/fk_lora_r16_b16_20k/checkpoint-11000 \\
   CUDA_VISIBLE_DEVICES=0 \\
   ./serve_hrdex_checkpoint_local.sh

3. ROS2 xArm terminal:
   cd /home/temp_id/xarm_ws
   sis
   ROS_NAMESPACE=right ros2 launch xarm_api xarm6_driver.launch.py \\
     robot_ip:=192.168.2.216 report_type:=dev joint_states_rate:=150 ros_namespace:=right

4. Bridge terminal:
   cd $WORKSPACE_ROOT/vla_inference
   PYTHON=python ./check_robot_inputs.sh
   PYTHON=python PARADEX_CAMERA=22645029 ./hrdex_dryrun.sh

EOF
