#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

: "${RLDX_SERVER_HOST:=127.0.0.1}"
: "${RLDX_SERVER_PORT:=5555}"
: "${INSTRUCTION:=pick up the cup}"
: "${PARADEX_LEFT_CAMERA:=}"
: "${PARADEX_RIGHT_CAMERA:=}"

uv run python xarm_inspire_rldx_bridge.py   --ros   --camera-source paradex   --dry-run   --server-host "$RLDX_SERVER_HOST"   --server-port "$RLDX_SERVER_PORT"   --instruction "$INSTRUCTION"   --paradex-root ../paradex   --paradex-left-camera-name "$PARADEX_LEFT_CAMERA"   --paradex-right-camera-name "$PARADEX_RIGHT_CAMERA"   --arm-state-topic /right/xarm/joint_states   --hand-state-topic /right/joint_states   --hand-command-topic /right/position_controller/commands
