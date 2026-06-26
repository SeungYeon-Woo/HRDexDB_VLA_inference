# HRDexDB VLA Inference Setup

Target setup:

```text
robot     : xArm6 + Inspire F1
arm       : ROS2 xarm_api
hand      : paradex direct Inspire controller
camera    : paradex FLIR stream, main camera 22645029
policy    : RLDX-1 server on the robot PC GPU
checkpoint: checkpoint-11000
```

Start with dry-run only. Do not move the robot until the logs look sane.

## 1. Robot PC Setup

Clone this repo and initialize everything:

```bash
mkdir -p ~/VCL/VLA_vanilla_test
cd ~/VCL/VLA_vanilla_test
git clone --recurse-submodules https://github.com/SeungYeon-Woo/HRDexDB_VLA_inference.git vla_inference
cd vla_inference
./setup_robot_pc.sh
```

This gives you:

```text
~/VCL/VLA_vanilla_test/vla_inference
~/VCL/VLA_vanilla_test/vla_inference/third_party/RLDX-1
~/VCL/VLA_vanilla_test/paradex
```

Set up the official RLDX-1 environment inside:

```text
~/VCL/VLA_vanilla_test/vla_inference/third_party/RLDX-1
```

Follow the official RLDX-1 repo instructions for its Python/CUDA environment.

## 2. Camera PC Setup

On the camera PC, install the HRDex stream script into paradex once:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
PARADEX_ROOT=/home/temp_id/paradex ./install_camera_stream_script.sh
```

Check camera names/serials if needed:

```bash
conda activate flir_env
cd /home/temp_id/paradex
python - <<'PY'
from paradex.utils.system import pc_name, get_camera_list
print("pc_name:", pc_name)
print("camera_list:", get_camera_list(pc_name))
PY
```

Main camera for this setup:

```text
22645029
```

Close SpinView before streaming.

Start the existing paradex camera daemon:

```bash
conda activate flir_env
cd /home/temp_id/paradex
python src/camera/server_daemon.py
```

In another terminal on the same camera PC, stream the main camera at 480x640:

```bash
conda activate flir_env
cd /home/temp_id/paradex
python src/capture/camera/stream_client_hrdex.py \
  --camera-names 22645029 \
  --fps 10
```

If restarting the daemon, kill only camera processes:

```bash
sudo pkill -f src/camera
```

Do not run `sudo pkill -f python` on capture PCs.

## 3. Start xArm ROS2 On Robot PC

```bash
cd /home/temp_id/xarm_ws
sis
ROS_NAMESPACE=right ros2 launch xarm_api xarm6_driver.launch.py \
  robot_ip:=192.168.2.216 \
  report_type:=dev \
  joint_states_rate:=150 \
  ros_namespace:=right
```

Check state topics in another ROS2 terminal:

```bash
sis
ros2 topic echo /right/xarm/joint_states --once
ros2 topic echo /right/xarm/robot_states --once
```

## 4. Start RLDX Server On Robot PC

Use the RLDX environment terminal:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
RLDX_MODEL_PATH=/research/ckpt/fk_lora_r16_b16_20k/fk_lora_r16_b16_20k/checkpoint-11000 \
CUDA_VISIBLE_DEVICES=0 \
./serve_hrdex_checkpoint_local.sh
```

Default server address:

```text
127.0.0.1:22610
```

## 5. Check Robot Inputs

Use a ROS2/paradex-capable terminal on the robot PC:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
PYTHON=python ./check_robot_inputs.sh
```

This checks xArm joint state, xArm EEF state, and Inspire F1 direct state.

## 6. Real Inference Dry-Run

No robot command is sent in this mode.

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
PYTHON=python \
PARADEX_CAMERA=22645029 \
INSTRUCTION="grasp the apple and release it" \
./hrdex_dryrun.sh
```

Check that logs show:

```text
video.main shape: [1, 4, 480, 640, 3]
camera lag is reasonable
arm_joint is 6D radians
eef_pose xyz is inside the xArm workspace
hand_joint / hand_cmd are finite and roughly in expected raw range
pred.eef_target is finite and not jumping wildly
```

## 7. Guarded Arm-Only Test

Only after dry-run looks sane:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
PYTHON=python \
PARADEX_CAMERA=22645029 \
./hrdex_execute.sh --no-dry-run --execute-arm --execution-horizon 1
```

The bridge uses only the first action step, clips workspace, and rate-limits motion.

Do not enable hand execution until hand scale/order/open-close direction is verified on the real hand.

## RLDX Contract

```text
input video.main  : (1, 4, H, W, 3), uint8 RGB, frames [-6, -4, -2, 0]
input arm_joint   : (1, 1, 6)
input hand_joint  : (1, 1, 6)
input eef_pose    : (1, 1, 9), [xyz_m, rot6d]
output eef_target : (1, 16, 9), absolute EEF target
output hand_cmd   : (1, 16, 6), absolute hand target
```
