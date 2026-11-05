#!/usr/bin/env bash
set -Eeuo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd -- "$BASE/../.." && pwd)"
SHARED="$ROOT/rl_lab_controller/g1_29dof_deployment"
cmake -S "$SHARED" -B "$SHARED/build_simulation" \
  -DTASK_ADAPTER_DIR="$BASE/controller" \
  -DG1_SIMULATION_JOINT_LIMITS=ON
cmake --build "$SHARED/build_simulation" --target g1_ctrl -j2
