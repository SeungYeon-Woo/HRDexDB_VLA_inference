# HRDexDB VLA Robot PC Bridge

This branch is the minimal robot-side bridge for running HRDexDB RLDX inference
from a separate inference PC.

It does not include RLDX-1, checkpoints, CUDA setup, or model-serving scripts.

## Runtime Topology

```text
camera PC     -> paradex camera stream
robot PC      -> camera receive + xArm ROS2 + Inspire direct + bridge client
inference PC  -> RLDX server + checkpoint-11000 + GPU
```

The robot PC sends observations to the inference PC and receives action chunks:

```text
video.main      (1, 4, 480, 640, 3), uint8 RGB
state.arm_joint (1, 1, 6)
state.hand_joint(1, 1, 6)
state.eef_pose  (1, 1, 9)

action.eef_target (1, 16, 9)
action.hand_cmd   (1, 16, 6)
```

## Robot PC Minimal Clone

```bash
mkdir -p ~/VCL/VLA_vanilla_test
cd ~/VCL/VLA_vanilla_test

git clone -b robot-pc-minimal \
  git@github.com:SeungYeon-Woo/HRDexDB_VLA_inference.git \
  vla_inference
```

This branch contains only the robot-side bridge files.

## Robot PC Dependencies

The robot PC needs its existing robot stack:

```text
ROS2
xarm_msgs
paradex
Inspire F1 direct controller dependencies
```

Python packages used by this bridge:

```bash
python -m pip install -r requirements-robot.txt
```

If these are already installed in the robot/paradex environment, do not create a
new environment. Use the environment that can import `rclpy`, `xarm_msgs`, and
`paradex`.

## Inference PC Server

Use the full `main` branch on the inference PC, not this minimal robot branch:

```bash
mkdir -p ~/VCL/VLA_vanilla_test
cd ~/VCL/VLA_vanilla_test
git clone --recurse-submodules \
  git@github.com:SeungYeon-Woo/HRDexDB_VLA_inference.git \
  vla_inference
```

Then run the RLDX server on the inference PC:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference

RLDX_MODEL_PATH=/home/seungyeon/VCL/checkpoints/HRDexDB_RLDX \
CUDA_VISIBLE_DEVICES=0 \
./serve_hrdex_checkpoint_local.sh
```

Expected:

```text
Server is ready and listening on tcp://127.0.0.1:22610
```

## Connect Robot PC To Inference PC

Recommended: SSH tunnel from robot PC to inference PC.

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
./ssh_tunnel_to_gpu.sh seungyeon@147.46.219.235
```

Then robot-side bridge uses:

```text
RLDX_SERVER_HOST=127.0.0.1
RLDX_SERVER_PORT=22610
```

Check:

```bash
nc -vz 127.0.0.1 22610
```

## Camera PC

Install the HRDex stream script into paradex once:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
PARADEX_ROOT=/home/temp_id/paradex ./install_camera_stream_script.sh
```

Start the camera daemon:

```bash
conda activate flir_env
cd /home/temp_id/paradex
python src/camera/server_daemon.py
```

Start streaming the main HRDexDB camera:

```bash
conda activate flir_env
cd /home/temp_id/paradex
python src/capture/camera/stream_client_hrdex.py \
  --camera-names 22645029 \
  --fps 10
```

Main camera:

```text
22645029
```

## xArm ROS2 On Robot PC

```bash
cd /home/temp_id/xarm_ws
sis
ROS_NAMESPACE=right ros2 launch xarm_api xarm6_driver.launch.py \
  robot_ip:=192.168.2.216 \
  report_type:=dev \
  joint_states_rate:=150 \
  ros_namespace:=right
```

Check:

```bash
sis
ros2 topic echo /right/xarm/joint_states --once
ros2 topic echo /right/xarm/robot_states --once
```

## Check Robot Inputs

Use a terminal that can import ROS2, xArm messages, and paradex:

```bash
cd ~/VCL/VLA_vanilla_test/vla_inference
PYTHON=python ./check_robot_inputs.sh
```

This checks:

```text
xArm joint state
xArm EEF state
Inspire F1 direct state
```

## Dry Run

No robot command is sent.

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
eef_pose xyz is inside xArm workspace
hand_joint / hand_cmd are finite and in a plausible raw range
pred.eef_target is finite and not jumping wildly
```

## Execution Warning

Start with dry-run only. Then use arm-only execution with:

```text
execution_horizon = 1
hand execution off
strong workspace clamp and rate limit
```

Do not execute hand commands until Inspire command order, scale, and open/close
direction have been verified on the real setup.
