# HRDexDB VLA Inference Setup

Target setup:

```text
robot     : xArm6 + Inspire F1
arm       : ROS2 xarm_api
hand      : paradex direct Inspire controller
camera    : paradex FLIR stream, main camera 22645029
policy    : RLDX-1 server on robot PC GPU or another inference PC
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

If RLDX inference runs on another local inference PC, the robot PC does not need the RLDX Python/CUDA environment or checkpoint. It only needs this bridge, ROS2, paradex access, and network access to the inference server.

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

## 3. Inference PC Setup

Do this on the separate local inference PC, not on the robot PC, if that PC will run RLDX.

Clone the same repo there:

```bash
mkdir -p ~/VCL/VLA_vanilla_test
cd ~/VCL/VLA_vanilla_test
git clone --recurse-submodules https://github.com/SeungYeon-Woo/HRDexDB_VLA_inference.git vla_inference
cd vla_inference
./setup_robot_pc.sh
```

Set up the official RLDX-1 environment inside:

```text
~/VCL/VLA_vanilla_test/vla_inference/third_party/RLDX-1
```

Follow the official RLDX-1 repo instructions for its Python/CUDA environment.

Download the checkpoint on the inference PC. The robot PC does not need the checkpoint when using remote inference.

The checkpoint is hosted on Hugging Face:

```text
yeon0857/HRDexDB_RLDX_checkpoint
```

Install the HF CLI and login once. If the repo is private, the token must have read permission.

```bash
python -m pip install -U huggingface_hub hf_transfer
huggingface-cli login
```

Download the checkpoint:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
./download_checkpoint.sh
```

Default checkpoint path after download:

```text
~/VCL/checkpoints/HRDexDB_RLDX/checkpoint-11000
```

## 4. Start xArm ROS2 On Robot PC

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

## 5. Start RLDX Server

Run this on the inference PC that has the GPU and checkpoint. In the remote setup, this is the separate local inference PC, not the robot PC.

Use the RLDX environment terminal:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
CUDA_VISIBLE_DEVICES=0 ./serve_hrdex_checkpoint_local.sh
```

If you downloaded the checkpoint somewhere else, override `RLDX_MODEL_PATH`:

```bash
RLDX_MODEL_PATH=/path/to/checkpoint-11000 \
CUDA_VISIBLE_DEVICES=0 \
./serve_hrdex_checkpoint_local.sh
```

Default server address:

```text
127.0.0.1:22610
```

### Connect Robot PC To The Inference PC

Recommended: use SSH tunneling from the robot PC to the inference PC.

On the inference PC, start RLDX with the default localhost bind:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
CUDA_VISIBLE_DEVICES=0 ./serve_hrdex_checkpoint_local.sh
```

On the robot PC, open a tunnel in a separate terminal:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
./ssh_tunnel_to_gpu.sh <USER@INFERENCE_PC_HOST>
```

Example:

```bash
./ssh_tunnel_to_gpu.sh seungyeon@192.168.0.104
```

Then the robot PC bridge still uses:

```text
RLDX_SERVER_HOST=127.0.0.1
RLDX_SERVER_PORT=22610
```

Check the tunnel from the robot PC:

```bash
nc -vz 127.0.0.1 22610
```

Alternative: bind the inference server to the LAN with `RLDX_BIND_HOST=0.0.0.0` and set `RLDX_SERVER_HOST=<INFERENCE_PC_IP>` on the robot PC. Use this only if firewall/network policy allows it.

## 6. Check Robot Inputs

Use a ROS2/paradex-capable terminal on the robot PC:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
PYTHON=python ./check_robot_inputs.sh
```

This checks xArm joint state, xArm EEF state, and Inspire F1 direct state.

## 7. Real Inference Dry-Run

No robot command is sent in this mode.

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
PYTHON=python \
RLDX_SERVER_HOST=127.0.0.1 \
RLDX_SERVER_PORT=22610 \
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

## 8. Guarded Arm-Only Test

Only after dry-run looks sane:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
PYTHON=python \
RLDX_SERVER_HOST=127.0.0.1 \
RLDX_SERVER_PORT=22610 \
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
