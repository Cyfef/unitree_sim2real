#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
TASK_NAME="${1:-supermimic}"
[[ "$TASK_NAME" =~ ^[A-Za-z0-9_-]+$ ]] || { echo 'Task name must be a folder name.' >&2; exit 2; }
TASK_DIR="$ROOT/$TASK_NAME"
[[ -f "$TASK_DIR/controller/main.cpp" && -d "$TASK_DIR/controller/src" ]] || {
  echo "Missing controller adapter in $TASK_DIR" >&2; exit 2;
}
python3 "$ROOT/scripts/check_assets.py" "$TASK_NAME"
PYTHON="${PYTHON:-python3.12}"
if [[ ! -x "$ROOT/.venv/bin/python" ]]; then "$PYTHON" -m venv "$ROOT/.venv"; fi
source "$ROOT/scripts/env.sh"
"$ROOT/.venv/bin/python" -m pip install -r "$ROOT/requirements.txt"
if [[ -f "$TASK_DIR/requirements.txt" ]]; then
  "$ROOT/.venv/bin/python" -m pip install -c "$ROOT/requirements.txt" -r "$TASK_DIR/requirements.txt"
fi
cmake --fresh -S "$ROOT/rl_lab_controller/g1_29dof_deployment" -B "$ROOT/rl_lab_controller/g1_29dof_deployment/build" -DTASK_ADAPTER_DIR="$TASK_DIR/controller" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ROOT/rl_lab_controller/g1_29dof_deployment/build" --target g1_ctrl -j2
printf 'Setup complete for %s. Follow %s/README.md for task checks.\n' "$TASK_NAME" "$TASK_DIR"
