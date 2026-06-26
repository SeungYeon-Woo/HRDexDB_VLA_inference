#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "usage: $0 <USER@GPU_SERVER_HOST>" >&2
  exit 2
fi

ssh -N -L 5555:127.0.0.1:5555 "$1"
