#!/usr/bin/env python3
"""Small RLDX policy-server client with no RLDX package dependency.

The robot PC only needs to send observations to a remote RLDX server and
receive action chunks back. Pulling the whole RLDX repo onto the robot PC just
for ``rldx.policy.server_client.PolicyClient`` is unnecessary, so this file
implements the tiny subset of that wire protocol used by the bridge.
"""

from __future__ import annotations

import io
from dataclasses import dataclass
from typing import Any

import msgpack
import numpy as np
import zmq


@dataclass
class RemoteModalityConfig:
    delta_indices: list[int]
    modality_keys: list[str]
    action_configs: list[Any] | None = None


class MsgSerializer:
    @staticmethod
    def to_bytes(data: Any) -> bytes:
        return msgpack.packb(data, default=MsgSerializer.encode_custom_classes, use_bin_type=True)

    @staticmethod
    def from_bytes(data: bytes) -> Any:
        return msgpack.unpackb(data, object_hook=MsgSerializer.decode_custom_classes, raw=False)

    @staticmethod
    def encode_custom_classes(obj: Any) -> Any:
        if isinstance(obj, RemoteModalityConfig):
            return {
                "__ModalityConfig_class__": True,
                "as_json": {
                    "delta_indices": obj.delta_indices,
                    "modality_keys": obj.modality_keys,
                    "action_configs": obj.action_configs,
                },
            }
        if isinstance(obj, np.ndarray):
            output = io.BytesIO()
            np.save(output, obj, allow_pickle=False)
            return {"__ndarray_class__": True, "as_npy": output.getvalue()}
        return obj

    @staticmethod
    def decode_custom_classes(obj: Any) -> Any:
        if not isinstance(obj, dict):
            return obj
        if "__ndarray_class__" in obj:
            return np.load(io.BytesIO(obj["as_npy"]), allow_pickle=False)
        if "__ModalityConfig_class__" in obj:
            cfg = obj["as_json"]
            return RemoteModalityConfig(
                delta_indices=[int(x) for x in cfg.get("delta_indices", [])],
                modality_keys=list(cfg.get("modality_keys", [])),
                action_configs=cfg.get("action_configs"),
            )
        return obj


class StandalonePolicyClient:
    def __init__(self, host: str, port: int, timeout_ms: int = 15000):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, int(timeout_ms))
        self.socket.setsockopt(zmq.SNDTIMEO, int(timeout_ms))
        self.socket.connect(f"tcp://{host}:{port}")

    def close(self) -> None:
        self.socket.close(0)
        self.context.term()

    def call_endpoint(
        self,
        endpoint: str,
        data: dict[str, Any] | None = None,
        *,
        requires_input: bool = True,
    ) -> Any:
        request: dict[str, Any] = {"endpoint": endpoint}
        if requires_input:
            request["data"] = data or {}
        self.socket.send(MsgSerializer.to_bytes(request))
        response = MsgSerializer.from_bytes(self.socket.recv())
        if isinstance(response, dict) and "error" in response:
            raise RuntimeError(f"RLDX server error: {response['error']}")
        return response

    def ping(self) -> bool:
        try:
            self.call_endpoint("ping", requires_input=False)
            return True
        except zmq.error.ZMQError:
            return False

    def get_modality_config(self) -> dict[str, RemoteModalityConfig]:
        return self.call_endpoint("get_modality_config", requires_input=False)

    def get_action(
        self,
        observation: dict[str, Any],
        *,
        session_id: str,
        reset_memory: bool,
    ) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
        response = self.call_endpoint(
            "get_action",
            {
                "observation": observation,
                "options": {
                    "session_ids": [session_id],
                    "reset_memory": [reset_memory],
                },
            },
        )
        action, info = tuple(response)
        return action, info
