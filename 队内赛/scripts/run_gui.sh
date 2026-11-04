#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${MJLAB_PYTHON:-/home/wangqikai/mjlab/.venv/bin/python}"
SCENARIO="manual"
ARGS=()
if (($# > 0)); then
  if [[ "$1" != -* ]]; then
    SCENARIO="$1"
    shift
  fi
  ARGS=("$@")
fi

exec "$PYTHON" "$ROOT/run_simulation.py" \
  --mode gui \
  --scenario "$SCENARIO" \
  "${ARGS[@]}"
