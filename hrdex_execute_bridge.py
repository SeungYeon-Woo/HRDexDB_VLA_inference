#!/usr/bin/env python3
"""Guarded real-robot execution bridge for HRDexDB RLDX checkpoint.

This follows the RLDX real-robot pattern: request a 16-step action chunk,
execute only the first N steps, then replan. Defaults are intentionally slow.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from hrdex_rldx_dryrun_bridge import (  # noqa: E402
    Latest,
    build_obs,
    camera_summary,
    euler_xyz_to_rotmat,
    frame_indices_from_deltas,
    select_video_frames,
    summarize_action,
    summarize_obs,
    xarm_pose6_to_eef_pose9,
)
from xarm_inspire_rldx_bridge import (  # noqa: E402
    ParadexCameraStream,
    ParadexDirectInspireHand,
    RLDXRemotePolicy,
)


def add_bool_arg(parser: argparse.ArgumentParser, name: str, *, default: bool, help: str | None = None) -> None:
    """Python 3.8-compatible replacement for argparse.BooleanOptionalAction."""
    dest = name.lstrip("-").replace("-", "_")
    group = parser.add_mutually_exclusive_group()
    group.add_argument(name, dest=dest, action="store_true", help=help)
    group.add_argument(f"--no-{name.lstrip('-')}", dest=dest, action="store_false")
    parser.set_defaults(**{dest: default})


HAND_RAW_LIMITS = np.array([
    [600.0, 1223.0],
    [1161.0, 1350.0],
    [973.0, 1740.0],
    [900.0, 1740.0],
    [900.0, 1740.0],
    [985.0, 1740.0],
], dtype=np.float32)

WORKSPACE_XYZ = np.array([
    [0.20, 0.65],
    [-0.40, 0.15],
    [0.07, 0.55],
], dtype=np.float32)

F1_RAW_LIMITS = np.array([1740.0, 1740.0, 1740.0, 1740.0, 1350.0, 1800.0], dtype=np.float32)
HRDEX_TO_DIRECT = np.array([5, 4, 3, 2, 1, 0], dtype=np.int64)


def rot6d_to_rotmat(rot6d: np.ndarray) -> np.ndarray:
    x = np.asarray(rot6d, dtype=np.float64).reshape(6)
    a1 = x[:3]
    a2 = x[3:6]
    b1 = a1 / max(np.linalg.norm(a1), 1e-8)
    a2_orth = a2 - np.dot(b1, a2) * b1
    b2 = a2_orth / max(np.linalg.norm(a2_orth), 1e-8)
    b3 = np.cross(b1, b2)
    return np.stack([b1, b2, b3], axis=1).astype(np.float32)


def rotmat_to_rotvec(rot: np.ndarray) -> np.ndarray:
    r = np.asarray(rot, dtype=np.float64).reshape(3, 3)
    cos_angle = np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(float(cos_angle))
    if angle < 1e-6:
        return np.zeros(3, dtype=np.float32)
    axis = np.array([
        r[2, 1] - r[1, 2],
        r[0, 2] - r[2, 0],
        r[1, 0] - r[0, 1],
    ], dtype=np.float64) / (2.0 * math.sin(angle))
    return (axis * angle).astype(np.float32)


def rotvec_to_rotmat(rotvec: np.ndarray) -> np.ndarray:
    v = np.asarray(rotvec, dtype=np.float64).reshape(3)
    theta = float(np.linalg.norm(v))
    if theta < 1e-8:
        return np.eye(3, dtype=np.float32)
    k = v / theta
    kx = np.array([
        [0.0, -k[2], k[1]],
        [k[2], 0.0, -k[0]],
        [-k[1], k[0], 0.0],
    ])
    r = np.eye(3) + math.sin(theta) * kx + (1.0 - math.cos(theta)) * (kx @ kx)
    return r.astype(np.float32)


def eef9_to_pose_aa(eef9: np.ndarray) -> np.ndarray:
    eef9 = np.asarray(eef9, dtype=np.float32).reshape(9)
    xyz_mm = eef9[:3] * 1000.0
    rotvec = rotmat_to_rotvec(rot6d_to_rotmat(eef9[3:9]))
    return np.concatenate([xyz_mm, rotvec]).astype(np.float32)


def pose6_to_eef9(pose6: np.ndarray) -> np.ndarray:
    pose6 = np.asarray(pose6, dtype=np.float32).reshape(6)
    xyz_m = pose6[:3] / 1000.0
    rot = euler_xyz_to_rotmat(pose6[3:6])
    return np.concatenate([xyz_m, rot[:, :2].reshape(6)]).astype(np.float32)


def guarded_eef_target(current_pose6: np.ndarray, pred_eef9: np.ndarray, *, max_xyz_step_m: float, max_rot_step_rad: float) -> np.ndarray:
    current_eef9 = pose6_to_eef9(current_pose6)
    cur_xyz = current_eef9[:3]
    tgt_xyz = np.clip(pred_eef9[:3], WORKSPACE_XYZ[:, 0], WORKSPACE_XYZ[:, 1])
    delta_xyz = np.clip(tgt_xyz - cur_xyz, -max_xyz_step_m, max_xyz_step_m)
    safe_xyz = cur_xyz + delta_xyz

    cur_r = euler_xyz_to_rotmat(np.asarray(current_pose6[3:6], dtype=np.float32))
    tgt_r = rot6d_to_rotmat(pred_eef9[3:9])
    rel_r = tgt_r @ cur_r.T
    rel_v = rotmat_to_rotvec(rel_r)
    rel_norm = float(np.linalg.norm(rel_v))
    if rel_norm > max_rot_step_rad:
        rel_v = rel_v / max(rel_norm, 1e-8) * max_rot_step_rad
    safe_r = rotvec_to_rotmat(rel_v) @ cur_r
    safe_rot6d = safe_r[:, :2].reshape(6)
    return np.concatenate([safe_xyz, safe_rot6d]).astype(np.float32)


def raw_to_f1_action(raw: np.ndarray) -> np.ndarray:
    # Policy outputs HRDexDB order:
    #   [thumb_1, thumb_2, index, middle, ring, little]
    # paradex direct Inspire controller expects normalized command in hardware
    # register order:
    #   [little, ring, middle, index, thumb_2, thumb_1]
    raw_hrdex = np.asarray(raw, dtype=np.float32).reshape(6)
    raw_direct = raw_hrdex[HRDEX_TO_DIRECT]
    return np.clip(raw_direct / F1_RAW_LIMITS * 1000.0, 0.0, 1000.0).astype(np.float64)


def guarded_hand_target(current_raw: np.ndarray, pred_raw: np.ndarray, *, max_step: float) -> np.ndarray:
    pred = np.asarray(pred_raw, dtype=np.float32).reshape(6)
    pred = np.clip(pred, HAND_RAW_LIMITS[:, 0], HAND_RAW_LIMITS[:, 1])
    cur = np.asarray(current_raw, dtype=np.float32).reshape(6)
    delta = np.clip(pred - cur, -max_step, max_step)
    return cur + delta


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--server-host", required=True)
    p.add_argument("--server-port", type=int, default=22610)
    p.add_argument("--instruction", required=True)
    p.add_argument("--session-id", default="hrdex_xarm_inspire_exec_0")
    p.add_argument("--control-hz", type=float, default=2.0)
    p.add_argument("--execution-horizon", type=int, default=1)
    p.add_argument("--request-timeout-ms", type=int, default=5000)
    p.add_argument("--paradex-root", default="../paradex")
    p.add_argument("--paradex-pc-list", default="")
    p.add_argument("--paradex-camera-name", default="")
    p.add_argument("--arm-state-topic", default="/right/xarm/joint_states")
    p.add_argument("--xarm-robot-states-topic", default="/right/xarm/robot_states")
    p.add_argument("--arm-joint-names", default="joint1,joint2,joint3,joint4,joint5,joint6")
    add_bool_arg(p, "--dry-run", default=True)
    add_bool_arg(p, "--execute-arm", default=False)
    add_bool_arg(p, "--execute-hand", default=False)
    add_bool_arg(
        p,
        "--allow-unverified-hand",
        default=False,
        help="Required together with --execute-hand until Inspire hand_cmd mapping is verified.",
    )
    p.add_argument("--max-xyz-step-m", type=float, default=0.01)
    p.add_argument("--max-rot-step-rad", type=float, default=0.05)
    p.add_argument("--max-hand-step", type=float, default=15.0)
    p.add_argument("--xarm-speed", type=float, default=0.0)
    p.add_argument("--xarm-acc", type=float, default=0.0)
    p.add_argument("--xarm-mvtime", type=float, default=0.0)
    p.add_argument("--log-jsonl", default="hrdex_execute_log.jsonl")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    if args.execution_horizon < 1 or args.execution_horizon > 16:
        raise ValueError("execution_horizon must be in [1, 16]")
    if args.execute_hand and not args.allow_unverified_hand:
        raise ValueError(
            "Refusing --execute-hand because Inspire hand_cmd mapping is not verified yet. "
            "Pass --allow-unverified-hand only after confirming unit/order/open-close direction."
        )

    import rclpy
    from sensor_msgs.msg import JointState
    from xarm_msgs.msg import RobotMsg
    from xarm_msgs.srv import MoveCartesian, SetInt16

    policy = RLDXRemotePolicy(args.server_host, args.server_port, args.request_timeout_ms)
    if not policy.ping():
        raise RuntimeError(f"RLDX server did not respond at {args.server_host}:{args.server_port}")
    modality_config = policy.get_modality_config()
    video_delta_indices = [int(x) for x in modality_config["video"].delta_indices]
    frame_queue_len = abs(min(frame_indices_from_deltas(video_delta_indices)))

    rclpy.init()
    node = rclpy.create_node("hrdex_execute_bridge")
    latest = Latest()
    latest_pose6: np.ndarray | None = None
    frames: deque[np.ndarray] = deque(maxlen=max(1, frame_queue_len))
    arm_joint_names = [x.strip() for x in args.arm_joint_names.split(",") if x.strip()]

    camera_stream = ParadexCameraStream(
        paradex_root=args.paradex_root,
        left_camera_name=args.paradex_camera_name,
        right_camera_name=args.paradex_camera_name,
        pc_list=ParadexCameraStream._parse_pc_list(args.paradex_pc_list),
        start_remote_stream=False,
        stream_fps=10,
    )
    hand = ParadexDirectInspireHand(paradex_root=args.paradex_root, tactile=False)

    set_mode = node.create_client(SetInt16, "/right/xarm/set_mode")
    set_state = node.create_client(SetInt16, "/right/xarm/set_state")
    set_servo_aa = node.create_client(MoveCartesian, "/right/xarm/set_servo_cartesian_aa")
    for cli, name in [(set_mode, "set_mode"), (set_state, "set_state"), (set_servo_aa, "set_servo_cartesian_aa")]:
        while not cli.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f"waiting for /right/xarm/{name}")

    def call_set_int(client, value: int) -> None:
        req = SetInt16.Request()
        req.data = int(value)
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=1.0)

    def send_xarm_pose_aa(pose_aa: np.ndarray) -> None:
        req = MoveCartesian.Request()
        req.pose = np.asarray(pose_aa, dtype=np.float32).tolist()
        req.speed = float(args.xarm_speed)
        req.acc = float(args.xarm_acc)
        req.mvtime = float(args.xarm_mvtime)
        req.is_tool_coord = False
        fut = set_servo_aa.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=0.5)

    if args.execute_arm and not args.dry_run:
        call_set_int(set_mode, 1)
        call_set_int(set_state, 0)

    def arm_cb(msg: JointState) -> None:
        if arm_joint_names:
            pos = dict(zip(msg.name, msg.position))
            latest.arm_joint = np.asarray([pos[n] for n in arm_joint_names], dtype=np.float32)
        else:
            latest.arm_joint = np.asarray(msg.position[:6], dtype=np.float32)

    def robot_state_cb(msg: RobotMsg) -> None:
        nonlocal latest_pose6
        latest_pose6 = np.asarray(msg.pose[:6], dtype=np.float32)
        latest.eef_pose = xarm_pose6_to_eef_pose9(latest_pose6)

    node.create_subscription(JointState, args.arm_state_topic, arm_cb, 10)
    node.create_subscription(RobotMsg, args.xarm_robot_states_topic, robot_state_cb, 10)

    log_f = open(args.log_jsonl, "a") if args.log_jsonl else None
    first_request = True
    period = 1.0 / args.control_hz

    print(
        "HRDexDB execute bridge started. "
        f"dry_run={args.dry_run} execute_arm={args.execute_arm} execute_hand={args.execute_hand} "
        f"execution_horizon={args.execution_horizon}"
    )

    try:
        while rclpy.ok():
            loop_start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)

            left_rgb, _right_rgb, cam_info = camera_stream.get_latest()
            if left_rgb is not None:
                latest.image_rgb = left_rgb
                frames.append(left_rgb)

            hand_raw = hand.get_raw_angle()
            if hand_raw is not None:
                latest.hand_joint = hand_raw

            if not latest.ready() or len(frames) < frame_queue_len or latest_pose6 is None:
                print(
                    "waiting obs: "
                    f"frames={len(frames)}/{frame_queue_len} arm={latest.arm_joint is not None} "
                    f"eef={latest.eef_pose is not None} hand={latest.hand_joint is not None}"
                )
                time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
                continue

            obs = build_obs(
                frames=frames,
                video_delta_indices=video_delta_indices,
                arm_joint=latest.arm_joint,
                hand_joint=latest.hand_joint,
                eef_pose=latest.eef_pose,
                instruction=args.instruction,
            )
            action, _info = policy.get_action(
                obs,
                session_id=args.session_id,
                reset_memory=first_request,
            )
            first_request = False

            obs_summary = summarize_obs(obs)
            act_summary = summarize_action(action)
            print({"obs": obs_summary, "action": act_summary, "camera": camera_summary(cam_info)})

            eef_chunk = np.asarray(action["eef_target"], dtype=np.float32)[0]
            hand_chunk = np.asarray(action["hand_cmd"], dtype=np.float32)[0]
            steps = min(args.execution_horizon, eef_chunk.shape[0], hand_chunk.shape[0])

            for step in range(steps):
                rclpy.spin_once(node, timeout_sec=0.0)
                if latest_pose6 is None or latest.hand_joint is None:
                    break
                safe_eef9 = guarded_eef_target(
                    latest_pose6,
                    eef_chunk[step],
                    max_xyz_step_m=args.max_xyz_step_m,
                    max_rot_step_rad=args.max_rot_step_rad,
                )
                safe_pose_aa = eef9_to_pose_aa(safe_eef9)
                safe_hand_raw = guarded_hand_target(
                    latest.hand_joint,
                    hand_chunk[step],
                    max_step=args.max_hand_step,
                )
                print(
                    f"exec[{step}] arm_pose_aa={np.round(safe_pose_aa, 4).tolist()} "
                    f"hand_raw={np.round(safe_hand_raw, 2).tolist()}"
                )
                if log_f is not None:
                    import json
                    log_f.write(json.dumps({
                        "time": time.time(),
                        "step": step,
                        "obs": obs_summary,
                        "action": act_summary,
                        "safe_pose_aa": safe_pose_aa.tolist(),
                        "safe_hand_raw": safe_hand_raw.tolist(),
                        "camera": camera_summary(cam_info),
                    }) + "\n")
                    log_f.flush()
                if not args.dry_run:
                    if args.execute_arm:
                        send_xarm_pose_aa(safe_pose_aa)
                    if args.execute_hand:
                        hand.move(raw_to_f1_action(safe_hand_raw))
                elapsed = time.monotonic() - loop_start
                target_elapsed = (step + 1) * period
                if elapsed < target_elapsed:
                    time.sleep(target_elapsed - elapsed)

    finally:
        if log_f is not None:
            log_f.close()
        hand.close()
        camera_stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
