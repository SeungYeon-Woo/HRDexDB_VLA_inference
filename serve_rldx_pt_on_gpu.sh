#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/../RLDX-1"

uv run python rldx/eval/run_rldx_server.py   --model-path RLWRLD/RLDX-1-PT   --embodiment-tag GENERAL_EMBODIMENT   --host "${RLDX_BIND_HOST:-127.0.0.1}"   --port "${RLDX_SERVER_PORT:-5555}"
