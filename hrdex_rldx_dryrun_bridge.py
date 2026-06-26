#!/usr/bin/env python3
"""Dry-run RLDX bridge for the HRDexDB xArm6 + Inspire checkpoint.

Expected checkpoint schema from the server-side note:
  video.main: (1, T, H, W, 3), uint8
  state.arm_joint: (1, 1, 6), float32
  state.hand_joint: (1, 1, 6), float32
  state.eef_pose: (1, 1, 9), float32
  language.annotation.human.action.task_description: [[instruction]]

Output is expected to contain:
  action.eef_target: (1, horizon, 9)
  action.hand_cmd: (1, horizon, 6)

This script intentionally does not command the robot. It only collects live
observation, calls the policy server, prints/logs action shapes and first step.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from datetime import datetime
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from xarm_inspire_rldx_bridge import (  # noqa: E402
    ParadexCameraStream,
    ParadexDirectInspireHand,
    RLDXRemotePolicy,
)


def euler_xyz_to_rotmat(rpy: np.ndarray) -> np.ndarray:
    r, p, y = [float(v) for v in rpy]
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float32)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float32)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float32)
    return rz @ ry @ rx


def xarm_pose6_to_eef_pose9(pose6: np.ndarray) -> np.ndarray:
    """Convert xArm pose [x_mm, y_mm, z_mm, roll, pitch, yaw] to xyz+rot6d.

    This assumes the checkpoint's 9D EEF pose is [xyz_m, rot6d]. Verify this
    against the HRDexDB preprocessing before executing any robot command.
    """
    pose6 = np.asarray(pose6, dtype=np.float32).reshape(6)
    xyz_m = pose6[:3] / 1000.0
    rot = euler_xyz_to_rotmat(pose6[3:6])
    rot6d = rot[:, :2].reshape(6)
    return np.concatenate([xyz_m, rot6d]).astype(np.float32)


def frame_indices_from_deltas(delta_indices: list[int]) -> list[int]:
    # RLDX delta 0 means current frame, -2 means two loop steps before current.
    # Python queue index for delta d is d - 1 for d <= 0: 0 -> -1, -2 -> -3.
    return [int(d) - 1 for d in delta_indices]


def select_video_frames(frames: deque[np.ndarray], delta_indices: list[int]) -> np.ndarray:
    py_indices = frame_indices_from_deltas(delta_indices)
    min_len = abs(min(py_indices))
    if len(frames) < min_len:
        raise RuntimeError(f"Need {min_len} queued frames for deltas {delta_indices}, have {len(frames)}")
    return np.stack([frames[i] for i in py_indices], axis=0).astype(np.uint8)


def array_summary(arr: np.ndarray) -> dict[str, Any]:
    a = np.asarray(arr)
    out: dict[str, Any] = {"shape": list(a.shape), "dtype": str(a.dtype)}
    if a.size:
        out.update(min=float(np.nanmin(a)), max=float(np.nanmax(a)), mean=float(np.nanmean(a)))
    return out


def parse_iso_lag_ms(timestamp: str | None) -> float | None:
    if not timestamp:
        return None
    try:
        ts = datetime.fromisoformat(timestamp)
        return (datetime.now(ts.tzinfo) - ts).total_seconds() * 1000.0
    except Exception:
        return None


def camera_summary(cam_info: dict[str, Any]) -> dict[str, Any]:
    out = dict(cam_info)
    ts = out.get("left_timestamp")
    lag_ms = parse_iso_lag_ms(ts)
    if lag_ms is not None:
        out["lag_ms"] = lag_ms
    return out


class Latest:
    def __init__(self) -> None:
        self.image_rgb: np.ndarray | None = None
        self.arm_joint: np.ndarray | None = None
        self.hand_joint: np.ndarray | None = None
        self.eef_pose: np.ndarray | None = None

    def ready(self) -> bool:
        return (
            self.image_rgb is not None
            and self.arm_joint is not None
            and self.hand_joint is not None
            and self.eef_pose is not None
        )


def build_obs(
    *,
    frames: deque[np.ndarray],
    video_delta_indices: list[int],
    arm_joint: np.ndarray,
    hand_joint: np.ndarray,
    eef_pose: np.ndarray,
    instruction: str,
) -> dict[str, Any]:
    video = select_video_frames(frames, video_delta_indices)
    return {
        "video": {"main": video[None]},
        "state": {
            "arm_joint": np.asarray(arm_joint, dtype=np.float32).reshape(1, 1, 6),
            "hand_joint": np.asarray(hand_joint, dtype=np.float32).reshape(1, 1, 6),
            "eef_pose": np.asarray(eef_pose, dtype=np.float32).reshape(1, 1, 9),
        },
        "language": {
            "annotation.human.action.task_description": [[instruction]],
        },
    }


def summarize_obs(obs: dict[str, Any]) -> dict[str, Any]:
    return {
        "video.main": array_summary(obs["video"]["main"]),
        "state.arm_joint": array_summary(obs["state"]["arm_joint"]),
        "state.hand_joint": array_summary(obs["state"]["hand_joint"]),
        "state.eef_pose": array_summary(obs["state"]["eef_pose"]),
    }


def summarize_action(action: dict[str, np.ndarray]) -> dict[str, Any]:
    out = {}
    for key, value in action.items():
        arr = np.asarray(value)
        item: dict[str, Any] = {
            "shape": list(arr.shape),
            "dtype": str(arr.dtype),
        }
        if arr.size:
            item.update(
                min=float(np.nanmin(arr)),
                max=float(np.nanmax(arr)),
                first_step=arr[0, 0].astype(float).tolist() if arr.ndim >= 3 else None,
            )
        out[key] = item
    return out


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--server-host", required=True)
    p.add_argument("--server-port", type=int, default=22610)
    p.add_argument("--instruction", required=True)
    p.add_argument("--session-id", default="hrdex_xarm_inspire_0")
    p.add_argument("--control-hz", type=float, default=2.0)
    p.add_argument("--video-t", type=int, default=0, help="0 means infer from server modality config")
    p.add_argument("--request-timeout-ms", type=int, default=5000)
    p.add_argument("--paradex-root", default="../paradex")
    p.add_argument("--paradex-pc-list", default="")
    p.add_argument("--paradex-camera-name", default="", help="Camera name/serial to map to video.main. Empty uses first image.")
    p.add_argument("--paradex-start-remote-stream", action=argparse.BooleanOptionalAction, default=False)
    p.add_argument("--paradex-stream-script", default="python src/capture/camera/stream_client_hrdex.py")
    p.add_argument("--paradex-stream-fps", type=int, default=10)
    p.add_argument("--arm-state-topic", default="/right/xarm/joint_states")
    p.add_argument("--xarm-robot-states-topic", default="/right/xarm/robot_states")
    p.add_argument("--arm-joint-names", default="joint1,joint2,joint3,joint4,joint5,joint6")
    p.add_argument("--hand-backend", choices=("paradex_direct", "none"), default="paradex_direct")
    p.add_argument("--inspire-direct-tactile", action=argparse.BooleanOptionalAction, default=False)
    p.add_argument("--log-jsonl", default="")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    import rclpy
    from sensor_msgs.msg import JointState
    from xarm_msgs.msg import RobotMsg

    policy = RLDXRemotePolicy(args.server_host, args.server_port, args.request_timeout_ms)
    if not policy.ping():
        raise RuntimeError(f"RLDX server did not respond at {args.server_host}:{args.server_port}")
    modality_config = policy.get_modality_config()
    server_video_delta_indices = list(modality_config["video"].delta_indices)
    if int(args.video_t) > 0:
        video_delta_indices = list(range(-(int(args.video_t) - 1), 1))
    else:
        video_delta_indices = [int(x) for x in server_video_delta_indices]
    video_t = len(video_delta_indices)
    frame_queue_len = abs(min(frame_indices_from_deltas(video_delta_indices)))
    print("Loaded server modality config:")
    print(f"  video keys          : {list(modality_config['video'].modality_keys)}")
    print(f"  video delta_indices : {server_video_delta_indices}")
    print(f"  client frame indices: {frame_indices_from_deltas(video_delta_indices)}")
    print(f"  state keys          : {list(modality_config['state'].modality_keys)}")
    print(f"  action keys         : {list(modality_config['action'].modality_keys)}")
    print(f"  action delta_indices: {list(modality_config['action'].delta_indices)}")

    rclpy.init()
    node = rclpy.create_node("hrdex_rldx_dryrun_bridge")
    latest = Latest()
    frames: deque[np.ndarray] = deque(maxlen=max(1, frame_queue_len))
    arm_joint_names = [x.strip() for x in args.arm_joint_names.split(",") if x.strip()]

    if args.paradex_start_remote_stream:
        root = Path(args.paradex_root).expanduser().resolve()
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))
        from paradex.io.capture_pc.ssh import run_script
        from paradex.io.camera_system.remote_camera_controller import remote_camera_controller

        pc_list = ParadexCameraStream._parse_pc_list(args.paradex_pc_list)
        run_script(args.paradex_stream_script, pc_list=pc_list)
        rcc = remote_camera_controller("hrdex_rldx_dryrun", pc_list=pc_list)
        rcc.start("stream", False, fps=args.paradex_stream_fps)

    camera_stream = ParadexCameraStream(
        paradex_root=args.paradex_root,
        left_camera_name=args.paradex_camera_name,
        right_camera_name=args.paradex_camera_name,
        pc_list=ParadexCameraStream._parse_pc_list(args.paradex_pc_list),
        start_remote_stream=False,
        stream_fps=args.paradex_stream_fps,
    )

    hand = None
    if args.hand_backend == "paradex_direct":
        hand = ParadexDirectInspireHand(
            paradex_root=args.paradex_root,
            tactile=args.inspire_direct_tactile,
        )
    else:
        latest.hand_joint = np.zeros(6, dtype=np.float32)

    def arm_cb(msg: JointState) -> None:
        if arm_joint_names:
            pos = dict(zip(msg.name, msg.position))
            latest.arm_joint = np.asarray([pos[n] for n in arm_joint_names], dtype=np.float32)
        else:
            latest.arm_joint = np.asarray(msg.position[:6], dtype=np.float32)

    def robot_state_cb(msg: RobotMsg) -> None:
        latest.eef_pose = xarm_pose6_to_eef_pose9(np.asarray(msg.pose[:6], dtype=np.float32))

    node.create_subscription(JointState, args.arm_state_topic, arm_cb, 10)
    node.create_subscription(RobotMsg, args.xarm_robot_states_topic, robot_state_cb, 10)

    log_f = open(args.log_jsonl, "a") if args.log_jsonl else None
    first_request = True
    period = 1.0 / args.control_hz

    print("HRDexDB RLDX dry-run bridge started. No robot command will be sent.")
    print(f"server={args.server_host}:{args.server_port}, video_t={video_t}, frame_queue_len={frame_queue_len}")

    try:
        while rclpy.ok():
            t0 = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)

            left_rgb, _right_rgb, cam_info = camera_stream.get_latest()
            if left_rgb is not None:
                latest.image_rgb = left_rgb
                frames.append(left_rgb)

            if hand is not None:
                hand_raw = hand.get_raw_angle()
                if hand_raw is not None:
                    latest.hand_joint = hand_raw

            if latest.ready() and len(frames) >= frame_queue_len:
                obs = build_obs(
                    frames=frames,
                    video_delta_indices=video_delta_indices,
                    arm_joint=latest.arm_joint,
                    hand_joint=latest.hand_joint,
                    eef_pose=latest.eef_pose,
                    instruction=args.instruction,
                )
                action, info = policy.get_action(
                    obs,
                    session_id=args.session_id,
                    reset_memory=first_request,
                )
                first_request = False
                obs_summary = summarize_obs(obs)
                summary = summarize_action(action)
                cam_summary = camera_summary(cam_info)
                print(json.dumps({"observation": obs_summary, "action": summary, "camera": cam_summary}, indent=2))
                if log_f is not None:
                    log_f.write(json.dumps({
                        "time": time.time(),
                        "arm_joint": latest.arm_joint.tolist(),
                        "hand_joint": latest.hand_joint.tolist(),
                        "eef_pose": latest.eef_pose.tolist(),
                        "observation": obs_summary,
                        "action": summary,
                        "camera": cam_summary,
                    }) + "\n")
                    log_f.flush()
            else:
                print(
                    "waiting obs: "
                    f"image={latest.image_rgb is not None} frames={len(frames)}/{frame_queue_len} "
                    f"arm={latest.arm_joint is not None} hand={latest.hand_joint is not None} "
                    f"eef={latest.eef_pose is not None}"
                )

            dt = time.monotonic() - t0
            time.sleep(max(0.0, period - dt))
    finally:
        if log_f is not None:
            log_f.close()
        if hand is not None:
            hand.close()
        camera_stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
