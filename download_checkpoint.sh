#!/usr/bin/env bash
set -euo pipefail

: "${HF_REPO_ID:=yeon0857/HRDexDB_RLDX_checkpoint}"
: "${HF_REPO_TYPE:=model}"
: "${CHECKPOINT_ROOT:=$HOME/VCL/checkpoints/HRDexDB_RLDX}"
: "${HF_INCLUDE:=}"

mkdir -p "$CHECKPOINT_ROOT"

if ! command -v huggingface-cli >/dev/null 2>&1; then
  echo "huggingface-cli is not installed. Install it with:" >&2
  echo "  python -m pip install -U huggingface_hub hf_transfer" >&2
  exit 1
fi

if ! huggingface-cli whoami >/dev/null 2>&1; then
  echo "Hugging Face auth is not configured. Login first:" >&2
  echo "  huggingface-cli login" >&2
  echo "Use a token that can read: $HF_REPO_ID" >&2
  exit 1
fi

cmd=(huggingface-cli download "$HF_REPO_ID" --repo-type "$HF_REPO_TYPE" --local-dir "$CHECKPOINT_ROOT")
if [ -n "$HF_INCLUDE" ]; then
  cmd+=(--include "$HF_INCLUDE")
fi

echo "Downloading $HF_REPO_ID to $CHECKPOINT_ROOT"
HF_HUB_ENABLE_HF_TRANSFER="${HF_HUB_ENABLE_HF_TRANSFER:-1}" "${cmd[@]}"

if [ -d "$CHECKPOINT_ROOT/checkpoint-11000" ]; then
  echo "Checkpoint ready: $CHECKPOINT_ROOT/checkpoint-11000"
else
  echo "Downloaded to: $CHECKPOINT_ROOT"
  echo "Note: checkpoint-11000 directory was not found at the top level. Set RLDX_MODEL_PATH manually if needed."
fi
