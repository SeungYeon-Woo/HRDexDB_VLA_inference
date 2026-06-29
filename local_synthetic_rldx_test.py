#!/usr/bin/env python3
"""Local synthetic HRDexDB input smoke test for an RLDX policy server.

This does not use ROS, paradex, a real camera, or a robot. It only verifies
that the local machine can build an HRDexDB-shaped observation, send it to the
RLDX server, and receive an action chunk.
"""

from __future__ import annotations

import argparse
import json
import time
from typing import Any

import numpy as np

from standalone_rldx_client import StandalonePolicyClient


def make_dummy_video(t: int, height: int, width: int, step: int) -> np.ndarray:
    y = np.linspace(0, 255, height, dtype=np.uint16)[:, None]
    x = np.linspace(0, 255, width, dtype=np.uint16)[None, :]
    frames = []
    for i in range(t):
        s = step + i
        frame = np.empty((height, width, 3), dtype=np.uint8)
        frame[..., 0] = ((x + s * 7) % 256).astype(np.uint8)
        frame[..., 1] = ((y + s * 11) % 256).astype(np.uint8)
        frame[..., 2] = (((x // 2 + y // 2) + s * 13) % 256).astype(np.uint8)
        col = (s * 17) % max(1, width - 80)
        row = (s * 9) % max(1, height - 80)
        frame[row:row + 80, col:col + 80] = np.array([255, 40, 40], dtype=np.uint8)
        frames.append(frame)
    return np.stack(frames, axis=0)[None]


def summarize_array(arr: np.ndarray) -> dict[str, Any]:
    arr = np.asarray(arr)
    out: dict[str, Any] = {"shape": list(arr.shape), "dtype": str(arr.dtype)}
    if arr.size:
        out.update(
            min=float(np.nanmin(arr)),
            max=float(np.nanmax(arr)),
            mean=float(np.nanmean(arr)),
        )
        if arr.ndim >= 3:
            out["first_step"] = arr[0, 0].astype(float).tolist()
    return out


def build_observation(*, t: int, height: int, width: int, instruction: str, step: int) -> dict[str, Any]:
    # Plausible HRDexDB FK-scale values. These are not meant to be physically
    # meaningful; they only test shape, dtype, serialization, preprocessing, and
    # action return.
    arm_joint = np.array([-0.55, -0.37, -0.45, 2.55, 0.84, 2.50], dtype=np.float32)
    hand_joint = np.array([1147.0, 1349.0, 1549.0, 1511.0, 1505.0, 1534.0], dtype=np.float32)
    eef_pose = np.array(
        [
            0.30,
            -0.18,
            0.31,
            0.108,
            0.868,
            0.485,
            -0.054,
            -0.482,
            0.875,
        ],
        dtype=np.float32,
    )
    return {
        "video": {"main": make_dummy_video(t, height, width, step)},
        "state": {
            "arm_joint": arm_joint.reshape(1, 1, 6),
            "hand_joint": hand_joint.reshape(1, 1, 6),
            "eef_pose": eef_pose.reshape(1, 1, 9),
        },
        "language": {
            "annotation.human.action.task_description": [[instruction]],
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=22610)
    parser.add_argument("--timeout-ms", type=int, default=30000)
    parser.add_argument("--instruction", default="grasp the apple and release it")
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--steps", type=int, default=1)
    parser.add_argument("--session-id", default="local_synthetic_hrdex_0")
    args = parser.parse_args()

    client = StandalonePolicyClient(args.host, args.port, args.timeout_ms)
    try:
        if not client.ping():
            raise RuntimeError(f"RLDX server did not respond at {args.host}:{args.port}")
        cfg = client.get_modality_config()
        video_delta = [int(x) for x in cfg["video"].delta_indices]
        t = len(video_delta)
        print("server modality:")
        print(f"  video keys          : {list(cfg['video'].modality_keys)}")
        print(f"  video delta_indices : {video_delta}")
        print(f"  state keys          : {list(cfg['state'].modality_keys)}")
        print(f"  action keys         : {list(cfg['action'].modality_keys)}")
        print(f"  action delta_indices: {list(cfg['action'].delta_indices)}")

        for step in range(args.steps):
            obs = build_observation(
                t=t,
                height=args.height,
                width=args.width,
                instruction=args.instruction,
                step=step,
            )
            t0 = time.monotonic()
            action, info = client.get_action(
                obs,
                session_id=args.session_id,
                reset_memory=(step == 0),
            )
            elapsed_ms = (time.monotonic() - t0) * 1000.0
            print(json.dumps({
                "step": step,
                "elapsed_ms": elapsed_ms,
                "observation": {
                    "video.main": summarize_array(obs["video"]["main"]),
                    "state.arm_joint": summarize_array(obs["state"]["arm_joint"]),
                    "state.hand_joint": summarize_array(obs["state"]["hand_joint"]),
                    "state.eef_pose": summarize_array(obs["state"]["eef_pose"]),
                },
                "action": {key: summarize_array(value) for key, value in action.items()},
                "info_keys": list(info.keys()) if isinstance(info, dict) else None,
            }, indent=2))
    finally:
        client.close()


if __name__ == "__main__":
    main()
