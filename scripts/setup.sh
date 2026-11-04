#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$ROOT/scripts/check_assets.py"
PYTHON="${PYTHON:-python3.12}"
if [[ ! -x "$ROOT/.venv/bin/python" ]]; then "$PYTHON" -m venv "$ROOT/.venv"; fi
export CYCLONEDDS_HOME="$ROOT/rl_lab_controller/cyclonedds"
"$ROOT/.venv/bin/python" -m pip install -r "$ROOT/supermimic/requirements.txt"
cmake --fresh -S "$ROOT/rl_lab_controller/g1_29dof_synced" -B "$ROOT/rl_lab_controller/g1_29dof_synced/build"
cmake --build "$ROOT/rl_lab_controller/g1_29dof_synced/build" --target g1_ctrl -j2
printf 'Setup complete. Run: cd supermimic && bash deploy.sh check\n'
