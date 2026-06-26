# VLA Inference for xArm6 + Inspire Hand

This folder is the runnable workspace for the RLDX-1 vanilla test.

Topology:

```text
Camera PC     : existing paradex FLIR daemon/stream
Robot PC      : ROS2 xArm + Inspire topics, this bridge
GPU server    : RLDX-1-PT PolicyServer
```

The bridge maps the current hardware into the OpenArm/Inspire-style RLDX schema:

```text
paradex left camera  -> video["camera_ego_left"]
paradex right camera -> video["camera_ego_right"]
xArm6 joint states   -> state["right_arm_joints"] padded to 7D
Inspire F1 states    -> state["right_hand_joints"]
left/neck states     -> zero-filled compatibility fields
```

Arm publishing is disabled by default. Keep it disabled until the RLDX output shape and action semantics are verified.

## GPU Server

Start the RLDX policy server on the GPU machine:

```bash
cd /path/to/VLA_vanilla_test/RLDX-1
uv run python rldx/eval/run_rldx_server.py \
  --model-path RLWRLD/RLDX-1-PT \
  --embodiment-tag GENERAL_EMBODIMENT \
  --host 127.0.0.1 \
  --port 5555
```

If the robot PC is on a different Wi-Fi, use SSH port forwarding from the robot PC:

```bash
ssh -N -L 5555:127.0.0.1:5555 <USER>@<GPU_SERVER_HOST>
```

Then the robot PC should use `--server-host 127.0.0.1 --server-port 5555`.

If both machines are on a routed/VPN network and the port is open, direct mode is also possible:

```bash
nc -vz <GPU_SERVER_IP_OR_HOSTNAME> 5555
```

In direct mode, run the GPU server with `--host 0.0.0.0` and pass `--server-host <GPU_SERVER_IP>` to the bridge.

## Robot PC Dry-Run

Run from this folder:

```bash
cd /home/seungyeon/VCL/VLA_vanilla_test/vla_inference
uv run python xarm_inspire_rldx_bridge.py \
  --ros \
  --camera-source paradex \
  --dry-run \
  --server-host 127.0.0.1 \
  --server-port 5555 \
  --instruction "pick up the cup" \
  --paradex-root ../paradex \
  --paradex-left-camera-name <LEFT_CAMERA_SERIAL_OR_NAME> \
  --paradex-right-camera-name <RIGHT_CAMERA_SERIAL_OR_NAME> \
  --arm-state-topic /right/xarm/joint_states \
  --hand-state-topic /right/joint_states \
  --hand-command-topic /right/position_controller/commands
```

If paradex stream is not already running, add:

```bash
  --paradex-start-remote-stream \
  --paradex-pc-list <CAMERA_PC_NAME> \
  --paradex-stream-fps 10
```

## Topic Defaults

Current defaults match the observed setup:

```text
xArm state      : /right/xarm/joint_states
Inspire state   : /right/joint_states
Inspire command : /right/position_controller/commands
```

## First Safety Rule

Use `--dry-run` first. In dry-run mode, no robot command is published; the bridge only logs camera availability, state mapping, action shape, and proposed hand/arm targets.

## HRDexDB Checkpoint Dry-Run

For the HRDexDB checkpoint described by the server team, use the dedicated dry-run bridge:

```text
video.main
state.arm_joint    shape 6
state.hand_joint   shape 6, raw Inspire angle units
state.eef_pose     shape 9
language.annotation.human.action.task_description

action.eef_target  shape 9
action.hand_cmd    shape 6, raw Inspire-style command
```

Start the checkpoint server on the GPU machine. If using SSH forwarding, bind the server to localhost there and forward port `22610` to the robot PC.

Robot PC dry-run, with no robot command output:

```bash
cd /home/seungyeon/VCL/VLA_vanilla_test/vla_inference
./hrdex_dryrun.sh
```

Optional environment variables:

```bash
RLDX_SERVER_HOST=127.0.0.1 \
RLDX_SERVER_PORT=22610 \
INSTRUCTION="grasp the apple and release it" \
PARADEX_CAMERA=<CAMERA_NAME_OR_SERIAL> \
LOG_JSONL=hrdex_dryrun_log.jsonl \
./hrdex_dryrun.sh
```

This bridge assumes:

```text
xArm state      : /right/xarm/joint_states
xArm EEF state  : /right/xarm/robot_states
Inspire state   : paradex direct serial/RS485 controller, converted to raw angle scale
Camera          : paradex camera stream
```

It converts xArm `robot_states.pose = [x_mm, y_mm, z_mm, roll, pitch, yaw]` into a tentative 9D `eef_pose = [xyz_m, rot6d]`. It also converts Inspire qpos radians back to raw RH56F1 angle units for `state.hand_joint`. Verify the EEF frame and rot6d convention match the HRDexDB preprocessing before any real robot execution.

## Camera Checks Before Trusting Inference


### HRDexDB 480x640 Camera Stream

The default paradex `src/capture/camera/stream_client.py` downsamples by 1/8, which sends roughly `192x256`. For checkpoint-11000, use the HRDexDB stream client so capture PCs send `480x640` JPEG frames:

```bash
conda activate flir_env
cd paradex
python src/capture/camera/stream_client_hrdex.py
```

Or let the dry-run bridge start it through paradex SSH helpers:

```bash
./hrdex_dryrun.sh --paradex-start-remote-stream
```

The robot-side bridge decodes JPEG with OpenCV and converts BGR to RGB before sending `video.main` to RLDX.


For checkpoint-11000, camera validation is required before robot motion:

```text
key          : video.main
format       : RGB uint8
shape        : (1, 4, H, W, 3)
delta indices: [-6, -4, -2, 0]
queue indices: [-7, -5, -3, -1]
training view: observation.images.main
training size: 480 x 640 x 3 before model preprocessing
```

The dry-run bridge logs observation summaries. Confirm:

```text
observation.video.main.shape is [1, 4, H, W, 3]
observation.video.main.dtype is uint8
camera.left_key is the intended HRDexDB main camera
image is RGB, not BGR
viewpoint matches the dataset's main camera view
frames update live and are not frozen
```

If the camera view is wrong, the model may still return plausible-looking numbers but they should not be trusted for robot execution.

