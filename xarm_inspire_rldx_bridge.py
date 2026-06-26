#!/usr/bin/env python3
"""RLDX-1 client bridge scaffold for xArm6 + Inspire Hand.

This script intentionally keeps ROS wiring configurable because topic/service
names vary by xArm/Inspire driver setup. It handles the stable parts:

1. Build RLDX OpenArm/Inspire-shaped observations from camera + joint state.
2. Query a remote RLDX PolicyServer over ZeroMQ.
3. Convert the returned action chunk into conservative xArm6 + hand commands.

Start with ``--dry-run``. Only disable it after verifying ROS topic names,
joint ordering, action shapes, and command limits.
"""

from __future__ import annotations

import argparse
import dataclasses
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np
import zmq

# Allow this script to run from the standalone vla_inference folder.
DEFAULT_RLDX_ROOT = Path(__file__).resolve().parents[1] / "RLDX-1"
if str(DEFAULT_RLDX_ROOT) not in sys.path:
    sys.path.insert(0, str(DEFAULT_RLDX_ROOT))

from rldx.policy.server_client import PolicyClient


DUMMY_IMAGE = np.zeros((256, 256, 3), dtype=np.uint8)


@dataclasses.dataclass
class BridgeConfig:
    server_host: str
    server_port: int
    instruction: str
    session_id: str
    control_hz: float
    execution_horizon: int
    request_timeout_ms: int
    dry_run: bool
    duplicate_right_image: bool
    arm_action_scale: float
    max_arm_delta_rad: float
    hand_min: float
    hand_max: float
    hand_output_scale: float
    arm_command_mode: str
    virtual_arm_joint_index: int
    virtual_arm_joint_value: float
    camera_source: str




class ParadexDirectInspireHand:
    """Direct Inspire F1 backend using paradex's RS485/serial controller.

    This is for setups where the Inspire hand is not exposed as ROS2 topics.
    It uses paradex.io.robot_controller.inspire_f1_controller.InspireF1Controller,
    whose move() API expects a 6D action in [0, 1000], with 1000 open and 0 closed.
    """

    def __init__(self, *, paradex_root: str, tactile: bool = False):
        root = Path(paradex_root).expanduser().resolve()
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))

        from paradex.utils.system import network_info
        from paradex.io.robot_controller.inspire_f1_controller import InspireF1Controller

        params = dict(network_info["inspire_f1"]["param"])
        params["tactile"] = tactile
        self.controller = InspireF1Controller(**params)

    @staticmethod
    def qpos_to_raw_angle(qpos: np.ndarray) -> np.ndarray:
        # Inverse of paradex.io.robot_controller.inspire_f1_controller._raw_to_qpos.
        # Returns RH56F1 raw angle units (0.1 deg), matching HRDexDB hand_joint/hand_cmd scale.
        qpos = np.asarray(qpos, dtype=np.float32).reshape(6)
        raw = np.zeros(6, dtype=np.float32)
        raw[:4] = (174.0 - qpos[:4] * 180.0 / np.pi) * 10.0
        raw[4] = (135.0 - qpos[4] * 180.0 / np.pi) * 10.0
        raw[5] = (180.0 - qpos[5] * 180.0 / np.pi) * 10.0
        return raw

    def get_qpos(self) -> np.ndarray | None:
        data = self.controller.get_data()
        qpos = data.get("qpos")
        if qpos is None:
            return None
        return np.asarray(qpos[:6], dtype=np.float32)

    def get_raw_angle(self) -> np.ndarray | None:
        qpos = self.get_qpos()
        if qpos is None:
            return None
        return self.qpos_to_raw_angle(qpos)

    def move(self, command: np.ndarray) -> None:
        self.controller.move(np.asarray(command, dtype=np.float64).reshape(6))

    def close(self) -> None:
        self.controller.end()

class RLDXRemotePolicy:
    def __init__(self, host: str, port: int, timeout_ms: int):
        self.client = PolicyClient(host=host, port=port, timeout_ms=timeout_ms, strict=False)
        # PolicyClient accepts timeout_ms but does not currently apply it to the
        # ZMQ socket. Set it here so robot-side code can fail closed.
        self.client.socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self.client.socket.setsockopt(zmq.SNDTIMEO, timeout_ms)

    def ping(self) -> bool:
        return self.client.ping()

    def get_modality_config(self) -> dict[str, Any]:
        return self.client.get_modality_config()

    def get_action(
        self,
        observation: dict[str, Any],
        *,
        session_id: str,
        reset_memory: bool,
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        return self.client.get_action(
            observation,
            options={
                "session_ids": [session_id],
                "reset_memory": [reset_memory],
            },
        )

class ParadexCameraStream:
    """Receives camera frames from paradex's existing camera server stack.

    Expected topology:
      camera PC: src/camera/server_daemon.py already running
      camera PC: src/capture/camera/stream_client.py publishes JPEGs
      robot PC : DataCollector receives latest JPEG per camera name

    If start_remote_stream=True, this class also asks paradex to start the
    remote stream mode, mirroring src/capture/camera/stream_remote.py.
    """

    def __init__(
        self,
        *,
        paradex_root: str,
        left_camera_name: str,
        right_camera_name: str,
        pc_list: list[str] | None,
        start_remote_stream: bool,
        stream_fps: int,
    ):
        root = Path(paradex_root).expanduser().resolve()
        if str(root) not in sys.path:
            sys.path.insert(0, str(root))

        import cv2
        from paradex.io.capture_pc.data_sender import DataCollector

        self.cv2 = cv2
        self.left_camera_name = left_camera_name
        self.right_camera_name = right_camera_name
        self.pc_list = pc_list
        self.remote_controller = None
        self.data_collector = DataCollector(pc_list=pc_list)
        self.data_collector.start()

        if start_remote_stream:
            from paradex.io.camera_system.remote_camera_controller import remote_camera_controller
            from paradex.io.capture_pc.ssh import run_script

            run_script("python src/capture/camera/stream_client.py", pc_list=pc_list)
            self.remote_controller = remote_camera_controller("rldx_bridge", pc_list=pc_list)
            self.remote_controller.start("stream", False, fps=stream_fps)

    @staticmethod
    def _parse_pc_list(value: str) -> list[str] | None:
        if not value:
            return None
        return [x.strip() for x in value.split(",") if x.strip()]

    def _decode_item(self, item: dict[str, Any]) -> tuple[np.ndarray, int, str]:
        image_bytes = item.get("data")
        if image_bytes is None:
            raise RuntimeError(f"Camera item has no JPEG data: {item.keys()}")
        encoded = np.frombuffer(image_bytes, np.uint8)
        bgr = self.cv2.imdecode(encoded, self.cv2.IMREAD_COLOR)
        if bgr is None:
            raise RuntimeError("Failed to decode paradex JPEG camera frame")
        rgb = bgr[..., ::-1].copy()
        return rgb, int(item.get("frame_id", 0)), str(item.get("timestamp", ""))

    def get_latest(self) -> tuple[np.ndarray | None, np.ndarray | None, dict[str, Any]]:
        all_data = self.data_collector.get_data()
        if not all_data:
            return None, None, {"available": []}

        left_key = self.left_camera_name
        right_key = self.right_camera_name
        if not left_key or not right_key:
            image_keys = [k for k, v in all_data.items() if v.get("type") == "image"]
            if not left_key and image_keys:
                left_key = image_keys[0]
            if not right_key and len(image_keys) > 1:
                right_key = image_keys[1]
            elif not right_key:
                right_key = left_key

        left_item = all_data.get(left_key)
        right_item = all_data.get(right_key)
        if left_item is None:
            return None, None, {"available": list(all_data.keys()), "missing": left_key}
        if right_item is None:
            right_item = left_item

        left_rgb, left_fid, left_ts = self._decode_item(left_item)
        right_rgb, right_fid, right_ts = self._decode_item(right_item)
        info = {
            "left_key": left_key,
            "right_key": right_key,
            "left_frame_id": left_fid,
            "right_frame_id": right_fid,
            "left_timestamp": left_ts,
            "right_timestamp": right_ts,
            "available": list(all_data.keys()),
        }
        return left_rgb, right_rgb, info

    def close(self) -> None:
        if self.remote_controller is not None:
            self.remote_controller.stop()
            self.remote_controller.end()
        self.data_collector.end()


class XArmInspireAdapter:
    """Maps xArm6 + Inspire state/actions to the OpenArm/Inspire convention.

    RLDX's bundled ``openarm_inspire_config`` uses:
      - neck_joints: 2
      - left_arm_joints: 7
      - right_arm_joints: 7
      - left_hand_joints: 6
      - right_hand_joints: 6

    xArm6 has only 6 arm joints, so we insert a virtual arm joint before
    inference and drop it after inference.
    """

    OPENARM_STATE_DIMS = {
        "neck_joints": 2,
        "left_arm_joints": 7,
        "right_arm_joints": 7,
        "left_hand_joints": 6,
        "right_hand_joints": 6,
    }

    def __init__(self, cfg: BridgeConfig):
        self.cfg = cfg

    def pad_xarm6_to_openarm7(self, q6: np.ndarray) -> np.ndarray:
        q6 = np.asarray(q6, dtype=np.float32).reshape(6)
        return np.insert(q6, self.cfg.virtual_arm_joint_index, self.cfg.virtual_arm_joint_value)

    def drop_openarm7_to_xarm6(self, q7: np.ndarray) -> np.ndarray:
        q7 = np.asarray(q7, dtype=np.float32).reshape(7)
        return np.delete(q7, self.cfg.virtual_arm_joint_index)

    def build_state(
        self,
        state_keys: list[str],
        xarm_joint_position: np.ndarray,
        inspire_position: np.ndarray,
    ) -> dict[str, np.ndarray]:
        state: dict[str, np.ndarray] = {}
        for key in state_keys:
            dim = self.OPENARM_STATE_DIMS.get(key)
            if key == "right_arm_joints":
                value = self.pad_xarm6_to_openarm7(xarm_joint_position)
            elif key == "right_hand_joints":
                value = np.asarray(inspire_position, dtype=np.float32).reshape(6)
            elif dim is not None:
                value = np.zeros(dim, dtype=np.float32)
            else:
                raise ValueError(
                    f"Unsupported state key '{key}'. Add its dimension/mapping to XArmInspireAdapter."
                )
            state[key] = value[None, None, :].astype(np.float32)
        return state

    def split_action(
        self,
        action_dict: dict[str, np.ndarray],
        current_xarm_q: np.ndarray,
        step_index: int,
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        arm_target = None
        hand_target = None

        if self.cfg.arm_command_mode == "disabled":
            arm_target = None
        elif self.cfg.arm_command_mode == "drop_virtual_joint" and "right_arm_joints" in action_dict:
            raw_arm7 = action_dict["right_arm_joints"][0, step_index]
            raw_arm6 = self.drop_openarm7_to_xarm6(raw_arm7)
            delta = np.clip(
                raw_arm6 * self.cfg.arm_action_scale,
                -self.cfg.max_arm_delta_rad,
                self.cfg.max_arm_delta_rad,
            )
            arm_target = np.asarray(current_xarm_q, dtype=np.float32).reshape(6) + delta
        else:
            raise ValueError(
                f"Unsupported arm_command_mode={self.cfg.arm_command_mode!r}. "
                "Use 'disabled' for safe dry tests or 'drop_virtual_joint' only for explicit experiments."
            )

        if "right_hand_joints" in action_dict:
            raw_hand = action_dict["right_hand_joints"][0, step_index] * self.cfg.hand_output_scale
            hand_target = np.clip(raw_hand, self.cfg.hand_min, self.cfg.hand_max).astype(np.float32)

        return arm_target, hand_target


class ObservationBuilder:
    def __init__(self, modality_config: dict[str, Any], adapter: XArmInspireAdapter, cfg: BridgeConfig):
        self.modality_config = modality_config
        self.adapter = adapter
        self.cfg = cfg
        video_cfg = modality_config["video"]
        self.video_keys = list(video_cfg.modality_keys)
        self.video_t = len(video_cfg.delta_indices)
        self.state_keys = list(modality_config["state"].modality_keys)
        self.language_key = modality_config["language"].modality_keys[0]
        self.left_image_history: deque[np.ndarray] = deque(maxlen=max(1, self.video_t))
        self.right_image_history: deque[np.ndarray] = deque(maxlen=max(1, self.video_t))

    @staticmethod
    def _validate_image(image_rgb: np.ndarray, name: str) -> np.ndarray:
        image_rgb = np.asarray(image_rgb)
        if image_rgb.ndim != 3 or image_rgb.shape[-1] != 3:
            raise ValueError(f"Expected {name} RGB image with shape (H, W, 3), got {image_rgb.shape}")
        return image_rgb.astype(np.uint8)

    def push_images(self, left_image_rgb: np.ndarray, right_image_rgb: np.ndarray | None = None) -> None:
        left = self._validate_image(left_image_rgb, "left")
        if right_image_rgb is None or self.cfg.duplicate_right_image:
            right = left
        else:
            right = self._validate_image(right_image_rgb, "right")
        self.left_image_history.append(left)
        self.right_image_history.append(right)

    def ready(self) -> bool:
        if len(self.left_image_history) < self.video_t:
            return False
        if len(self.video_keys) > 1 and len(self.right_image_history) < self.video_t:
            return False
        return True

    def build(
        self,
        xarm_joint_position: np.ndarray,
        inspire_position: np.ndarray,
        instruction: str,
        right_image_rgb: np.ndarray | None = None,
    ) -> dict[str, Any]:
        del right_image_rgb  # images are buffered through push_images() before build().
        if not self.ready():
            raise RuntimeError("Not enough image history for the policy yet.")

        left_frames = np.stack(list(self.left_image_history)[-self.video_t :], axis=0)
        right_frames = np.stack(list(self.right_image_history)[-self.video_t :], axis=0)
        video = {}
        for i, key in enumerate(self.video_keys):
            selected = left_frames if i == 0 else right_frames
            video[key] = selected[None].astype(np.uint8)

        return {
            "video": video,
            "state": self.adapter.build_state(
                self.state_keys,
                xarm_joint_position=xarm_joint_position,
                inspire_position=inspire_position,
            ),
            "language": {self.language_key: [[instruction]]},
        }


class LatestState:
    def __init__(self):
        self.left_image_rgb: np.ndarray | None = None
        self.right_image_rgb: np.ndarray | None = None
        self.xarm_q: np.ndarray | None = None
        self.hand_q: np.ndarray | None = None

    def ready(self) -> bool:
        return self.left_image_rgb is not None and self.xarm_q is not None and self.hand_q is not None


def _parse_float_list(value: str, expected_len: int, name: str) -> np.ndarray:
    arr = np.asarray([float(x) for x in value.split(",") if x != ""], dtype=np.float32)
    if arr.shape != (expected_len,):
        raise ValueError(f"{name} must contain {expected_len} comma-separated floats, got {value!r}")
    return arr


def run_dry_loop(policy: RLDXRemotePolicy, cfg: BridgeConfig) -> None:
    modality_config = policy.get_modality_config()
    adapter = XArmInspireAdapter(cfg)
    builder = ObservationBuilder(modality_config, adapter, cfg)

    xarm_q = np.zeros(6, dtype=np.float32)
    hand_q = np.zeros(6, dtype=np.float32)
    for _ in range(builder.video_t):
        builder.push_images(DUMMY_IMAGE, DUMMY_IMAGE)

    print("Dry-run observation contract")
    print(f"  video keys   : {builder.video_keys}, T={builder.video_t}")
    print(f"  state keys   : {builder.state_keys}")
    print(f"  language key : {builder.language_key}")

    obs = builder.build(xarm_q, hand_q, cfg.instruction)
    action, info = policy.get_action(obs, session_id=cfg.session_id, reset_memory=True)
    print("Received action")
    for key, value in action.items():
        print(f"  {key}: shape={value.shape}, dtype={value.dtype}, min={value.min():.4f}, max={value.max():.4f}")
    arm_target, hand_target = adapter.split_action(action, xarm_q, step_index=0)
    print(f"Adapted arm target : {arm_target}")
    print(f"Adapted hand target: {hand_target}")
    if info:
        print(f"Info keys: {list(info.keys())}")


def run_ros_loop(policy: RLDXRemotePolicy, cfg: BridgeConfig, args: argparse.Namespace) -> None:
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    rclpy.init()
    node = rclpy.create_node("xarm_inspire_rldx_bridge")
    latest = LatestState()
    paradex_stream = None
    if cfg.camera_source == "paradex":
        paradex_stream = ParadexCameraStream(
            paradex_root=args.paradex_root,
            left_camera_name=args.paradex_left_camera_name,
            right_camera_name=args.paradex_right_camera_name,
            pc_list=ParadexCameraStream._parse_pc_list(args.paradex_pc_list),
            start_remote_stream=args.paradex_start_remote_stream,
            stream_fps=args.paradex_stream_fps,
        )

    direct_hand = None
    if args.hand_backend == "paradex_direct":
        direct_hand = ParadexDirectInspireHand(
            paradex_root=args.paradex_root,
            tactile=args.inspire_direct_tactile,
        )
    elif args.hand_backend == "none":
        latest.hand_q = np.zeros(6, dtype=np.float32)

    modality_config = policy.get_modality_config()
    adapter = XArmInspireAdapter(cfg)
    builder = ObservationBuilder(modality_config, adapter, cfg)

    arm_joint_names = [x.strip() for x in args.arm_joint_names.split(",") if x.strip()]

    if cfg.camera_source == "ros":
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image

        cv_bridge = CvBridge()

        def left_image_cb(msg: Image) -> None:
            bgr = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            latest.left_image_rgb = bgr[..., ::-1].copy()

        def right_image_cb(msg: Image) -> None:
            bgr = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            latest.right_image_rgb = bgr[..., ::-1].copy()

        node.create_subscription(Image, args.left_image_topic, left_image_cb, 10)
        if args.right_image_topic:
            node.create_subscription(Image, args.right_image_topic, right_image_cb, 10)

    def arm_state_cb(msg: JointState) -> None:
        if arm_joint_names:
            positions = dict(zip(msg.name, msg.position))
            latest.xarm_q = np.asarray([positions[n] for n in arm_joint_names], dtype=np.float32)
        else:
            latest.xarm_q = np.asarray(msg.position[:6], dtype=np.float32)

    def hand_state_cb(msg: JointState) -> None:
        latest.hand_q = np.asarray(msg.position[:6], dtype=np.float32)

    node.create_subscription(JointState, args.arm_state_topic, arm_state_cb, 10)
    if args.hand_backend == "ros_topic":
        node.create_subscription(JointState, args.hand_state_topic, hand_state_cb, 10)

    arm_pub = node.create_publisher(JointTrajectory, args.arm_command_topic, 10)
    hand_pub = None
    if args.hand_backend == "ros_topic":
        hand_pub = node.create_publisher(Float64MultiArray, args.hand_command_topic, 10)

    action_chunk: dict[str, np.ndarray] | None = None
    action_step = 0
    first_request = True
    period = 1.0 / cfg.control_hz

    print("ROS bridge is running. Keep --dry-run until command topics and scaling are verified.")
    while rclpy.ok():
        start = time.monotonic()
        rclpy.spin_once(node, timeout_sec=0.0)

        if paradex_stream is not None:
            left_rgb, right_rgb, cam_info = paradex_stream.get_latest()
            if left_rgb is not None:
                latest.left_image_rgb = left_rgb
                latest.right_image_rgb = right_rgb
                if action_chunk is None:
                    node.get_logger().info(
                        f"paradex cameras left={cam_info.get('left_key')} "
                        f"right={cam_info.get('right_key')} "
                        f"frames=({cam_info.get('left_frame_id')}, {cam_info.get('right_frame_id')})"
                    )

        if direct_hand is not None:
            hand_q = direct_hand.get_qpos()
            if hand_q is not None:
                latest.hand_q = hand_q

        if latest.ready():
            builder.push_images(latest.left_image_rgb, latest.right_image_rgb)

        if latest.ready() and builder.ready() and (action_chunk is None or action_step >= cfg.execution_horizon):
            obs = builder.build(
                latest.xarm_q,
                latest.hand_q,
                cfg.instruction,
                right_image_rgb=latest.right_image_rgb,
            )
            try:
                action_chunk, _ = policy.get_action(
                    obs,
                    session_id=cfg.session_id,
                    reset_memory=first_request,
                )
                first_request = False
                action_step = 0
            except Exception as exc:
                node.get_logger().error(f"RLDX request failed; holding current command: {exc}")
                action_chunk = None

        if action_chunk is not None and latest.xarm_q is not None:
            arm_target, hand_target = adapter.split_action(action_chunk, latest.xarm_q, action_step)
            action_step += 1

            if cfg.dry_run:
                node.get_logger().info(f"dry arm={arm_target} hand={hand_target}")
            else:
                if arm_target is not None:
                    msg = JointTrajectory()
                    msg.joint_names = arm_joint_names
                    point = JointTrajectoryPoint()
                    point.positions = arm_target.astype(float).tolist()
                    point.time_from_start.sec = 1
                    msg.points = [point]
                    arm_pub.publish(msg)
                if hand_target is not None:
                    if args.hand_backend == "ros_topic" and hand_pub is not None:
                        hand_pub.publish(Float64MultiArray(data=hand_target.astype(float).tolist()))
                    elif args.hand_backend == "paradex_direct" and direct_hand is not None:
                        direct_hand.move(hand_target)

        elapsed = time.monotonic() - start
        if elapsed < period:
            time.sleep(period - elapsed)

    if paradex_stream is not None:
        paradex_stream.close()
    if direct_hand is not None:
        direct_hand.close()
    node.destroy_node()
    rclpy.shutdown()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--server-host", required=True)
    parser.add_argument("--server-port", type=int, default=5555)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--session-id", default="xarm_inspire_0")
    parser.add_argument("--control-hz", type=float, default=5.0)
    parser.add_argument("--execution-horizon", type=int, default=4)
    parser.add_argument("--request-timeout-ms", type=int, default=2000)
    parser.add_argument("--dry-run", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--duplicate-right-image", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--arm-action-scale", type=float, default=0.05)
    parser.add_argument("--max-arm-delta-rad", type=float, default=0.03)
    parser.add_argument("--hand-min", type=float, default=0.0)
    parser.add_argument("--hand-max", type=float, default=1.0)
    parser.add_argument("--hand-output-scale", type=float, default=1.0)
    parser.add_argument(
        "--arm-command-mode",
        choices=("disabled", "drop_virtual_joint"),
        default="disabled",
        help=(
            "disabled is the safe default. drop_virtual_joint treats the model's 7-DoF "
            "right_arm_joints output as a small joint delta after dropping one virtual joint; "
            "this is only a crude smoke-test retargeting, not kinematically correct."
        ),
    )
    parser.add_argument("--virtual-arm-joint-index", type=int, default=6)
    parser.add_argument("--virtual-arm-joint-value", type=float, default=0.0)
    parser.add_argument("--ros", action="store_true")
    parser.add_argument("--camera-source", choices=("ros", "paradex"), default="ros")
    parser.add_argument("--left-image-topic", "--image-topic", dest="left_image_topic", default="/camera_left/color/image_raw")
    parser.add_argument("--right-image-topic", "--secondary-image-topic", dest="right_image_topic", default="/camera_right/color/image_raw")
    parser.add_argument("--arm-state-topic", default="/right/xarm/joint_states")
    parser.add_argument("--hand-state-topic", default="/right/joint_states")
    parser.add_argument("--arm-command-topic", default="/right/joint_trajectory_controller/joint_trajectory")
    parser.add_argument("--hand-command-topic", default="/right/position_controller/commands")
    parser.add_argument("--hand-backend", choices=("ros_topic", "paradex_direct", "none"), default="ros_topic")
    parser.add_argument("--inspire-direct-tactile", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--paradex-root", default="../paradex")
    parser.add_argument("--paradex-pc-list", default="", help="Comma-separated paradex camera PC names. Empty uses paradex config.")
    parser.add_argument("--paradex-left-camera-name", default="", help="Camera serial/name for RLDX camera_ego_left. Empty chooses first received image.")
    parser.add_argument("--paradex-right-camera-name", default="", help="Camera serial/name for RLDX camera_ego_right. Empty chooses second received image or duplicates left.")
    parser.add_argument("--paradex-start-remote-stream", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--paradex-stream-fps", type=int, default=10)
    parser.add_argument(
        "--arm-joint-names",
        default="joint1,joint2,joint3,joint4,joint5,joint6",
        help="Comma-separated xArm6 joint names in command/state order. Empty means first 6 positions.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cfg = BridgeConfig(
        server_host=args.server_host,
        server_port=args.server_port,
        instruction=args.instruction,
        session_id=args.session_id,
        control_hz=args.control_hz,
        execution_horizon=args.execution_horizon,
        request_timeout_ms=args.request_timeout_ms,
        dry_run=args.dry_run,
        duplicate_right_image=args.duplicate_right_image,
        arm_action_scale=args.arm_action_scale,
        max_arm_delta_rad=args.max_arm_delta_rad,
        hand_min=args.hand_min,
        hand_max=args.hand_max,
        hand_output_scale=args.hand_output_scale,
        arm_command_mode=args.arm_command_mode,
        virtual_arm_joint_index=args.virtual_arm_joint_index,
        virtual_arm_joint_value=args.virtual_arm_joint_value,
        camera_source=args.camera_source,
    )
    policy = RLDXRemotePolicy(cfg.server_host, cfg.server_port, cfg.request_timeout_ms)
    if not policy.ping():
        raise RuntimeError(f"RLDX server did not respond at {cfg.server_host}:{cfg.server_port}")

    if args.ros:
        run_ros_loop(policy, cfg, args)
    else:
        run_dry_loop(policy, cfg)


if __name__ == "__main__":
    main()
