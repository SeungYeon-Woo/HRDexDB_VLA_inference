# HRDexDB RLDX-1 Inference Bridge

This folder contains the robot-side bridge for running the HRDexDB RLDX-1 checkpoint on:

```text
robot   : xArm6 + Inspire F1
camera  : one HRDexDB main-view FLIR stream
arm I/O : ROS2 xarm_api
hand I/O: paradex direct Inspire controller
policy  : RLDX-1 ZMQ inference server
```

The first target is real inference dry-run. Robot motion is disabled by default.


## Setup Summary

What you need to do:

1. Robot PC: clone/setup official `RLDX-1`.
2. Robot PC: clone/setup this `vla_inference` repo.
3. Camera PC: install/run `stream_client_hrdex.py` inside paradex.

For a fresh robot PC workspace:

```bash
mkdir -p ~/VCL/VLA_vanilla_test
cd ~/VCL/VLA_vanilla_test
git clone https://github.com/SeungYeon-Woo/HRDexDB_VLA_inference.git vla_inference
cd vla_inference
./setup_robot_pc.sh
```

The setup script clones:

```text
~/VCL/VLA_vanilla_test/RLDX-1
~/VCL/VLA_vanilla_test/vla_inference
~/VCL/VLA_vanilla_test/paradex
```

It also keeps a copy of the camera stream script at:

```text
vla_inference/camera/stream_client_hrdex.py
```

If you already cloned `RLDX-1` or `paradex`, the script leaves them in place.

On a camera PC with paradex checked out, install the HRDex stream script with:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
PARADEX_ROOT=/home/temp_id/paradex ./install_camera_stream_script.sh
```

Then run it from the camera PC paradex checkout:

```bash
conda activate flir_env
cd /home/temp_id/paradex
python src/capture/camera/stream_client_hrdex.py --fps 10
```

## Runtime Split

Use two terminals with separate environments.

Terminal A runs the RLDX server. This environment needs CUDA, the RLDX-1 codebase, and the checkpoint.

Terminal B runs the robot bridge. This environment needs ROS2 sourced, access to paradex, and Python deps for the client code. It does not load the checkpoint.

If the RTX 4090 and robot are on the same PC, keep the server bound to localhost:

```text
RLDX server: 127.0.0.1:22610
bridge     : connects to 127.0.0.1:22610
```

No SSH tunnel is needed in that case.

## RLDX Server

Start the HRDexDB checkpoint server:

```bash
cd /home/seungyeon/VCL/VLA_vanilla_test/vla_inference

RLDX_ROOT=/research/RLDX-1 \
RLDX_MODEL_PATH=/research/ckpt/fk_lora_r16_b16_20k/fk_lora_r16_b16_20k/checkpoint-11000 \
CUDA_VISIBLE_DEVICES=0 \
./serve_hrdex_checkpoint_local.sh
```

If the RLDX repo is cloned somewhere else, set `RLDX_ROOT` and optionally `RLDX_PYTHON`:

```bash
RLDX_ROOT=/home/seungyeon/VCL/VLA_vanilla_test/RLDX-1 \
RLDX_PYTHON=/home/seungyeon/VCL/VLA_vanilla_test/RLDX-1/.venv/bin/python \
RLDX_MODEL_PATH=/research/ckpt/fk_lora_r16_b16_20k/fk_lora_r16_b16_20k/checkpoint-11000 \
./serve_hrdex_checkpoint_local.sh
```

Expected server settings:

```text
embodiment-tag: GENERAL_EMBODIMENT
host          : 127.0.0.1
port          : 22610
```

## Camera Stream

Use one camera: the HRDexDB `main` view or the physical view closest to it.

On the camera PC, close SpinView first because Spinnaker camera access is usually exclusive. Then run:

```bash
conda activate flir_env
cd paradex
python src/camera/server_daemon.py
```

In another camera-PC terminal:

```bash
conda activate flir_env
cd paradex
python src/capture/camera/stream_client_hrdex.py
```

`stream_client_hrdex.py` sends 480x640 JPEG frames. The robot bridge decodes them and converts BGR to RGB before sending `video.main` to RLDX.

## xArm ROS2 Driver

On the robot PC:

```bash
cd /home/temp_id/xarm_ws
sis
ROS_NAMESPACE=right ros2 launch xarm_api xarm6_driver.launch.py \
  robot_ip:=192.168.2.216 \
  report_type:=dev \
  joint_states_rate:=150 \
  ros_namespace:=right
```

Expected ROS state topics/services include:

```text
/right/xarm/joint_states
/right/xarm/robot_states
/right/xarm/set_servo_cartesian_aa
/right/xarm/set_mode
/right/xarm/set_state
```

## Input Contract

The checkpoint expects:

```text
video.main
  shape : (1, 4, H, W, 3)
  dtype : uint8 RGB
  frames: [-6, -4, -2, 0]
  queue : [-7, -5, -3, -1]

state.arm_joint
  shape : (1, 1, 6)
  source: /right/xarm/joint_states.position[:6]
  unit  : radians

state.eef_pose
  shape : (1, 1, 9)
  source: /right/xarm/robot_states.pose
  format: [x_m, y_m, z_m, rot6d]

state.hand_joint
  shape : (1, 1, 6)
  source: paradex direct Inspire F1 state
```

The checkpoint outputs a 16-step chunk:

```text
action.eef_target: (1, 16, 9), absolute EEF target
action.hand_cmd  : (1, 16, 6), absolute Inspire-style target
```

This checkpoint is not an RTC checkpoint. Start with `execution_horizon=1`, then only increase after logs and motion look stable.

## Check Inputs

In a ROS2/paradex-capable terminal on the robot PC:

```bash
cd /home/seungyeon/VCL/VLA_vanilla_test/vla_inference
PYTHON=python ./check_robot_inputs.sh
```

This checks xArm joint state, xArm EEF state, and three Inspire direct state samples.

If the ROS/paradex environment uses a specific Python, pass it explicitly:

```bash
PYTHON=/path/to/python ./check_robot_inputs.sh
```

## Real Inference Dry-Run

Run this before any robot motion:

```bash
cd /home/seungyeon/VCL/VLA_vanilla_test/vla_inference
PYTHON=python \
RLDX_SERVER_HOST=127.0.0.1 \
RLDX_SERVER_PORT=22610 \
PARADEX_CAMERA=<MAIN_CAMERA_NAME_OR_SERIAL> \
INSTRUCTION="grasp the apple and release it" \
./hrdex_dryrun.sh
```

Dry-run sends observations to the RLDX server and logs predicted actions. It does not command the robot.

Confirm in the log:

```text
video.main shape is [1, 4, 480, 640, 3] or equivalent H/W
video.main dtype is uint8
camera lag is reasonable and frames are live
arm_joint values are radians
state.eef_pose xyz is in the expected xArm workspace
pred.eef_target xyz is roughly within the workspace
pred.hand_cmd is roughly in the observed raw range, not NaN or huge
```

## Guarded Arm Test

The execution wrapper is still dry-run by default:

```bash
./hrdex_execute.sh
```

To command arm motion, use arm-only mode after dry-run logs look reasonable:

```bash
PYTHON=python \
RLDX_SERVER_HOST=127.0.0.1 \
RLDX_SERVER_PORT=22610 \
PARADEX_CAMERA=<MAIN_CAMERA_NAME_OR_SERIAL> \
./hrdex_execute.sh --no-dry-run --execute-arm --execution-horizon 1
```

The bridge clips workspace, limits translation step size, limits rotation step size, and only uses the front of the action chunk.

Do not enable hand motion yet. Hand mapping/order/open-close direction still need verification against the Inspire collection code and raw data. The code requires `--allow-unverified-hand` before it will send hand commands.

## Git Push

If `git push` fails with HTTPS auth, authenticate first:

```bash
gh auth login
git push -u origin main
```

Or switch the remote to SSH after registering an SSH key with GitHub.
