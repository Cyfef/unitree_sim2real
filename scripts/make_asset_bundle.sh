#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:?give an output .tar.gz path}"
python3 "$ROOT/scripts/check_assets.py"
mkdir -p "$(dirname -- "$OUT")"
tar -C "$ROOT" -czf "$OUT" \
  models/meshes \
  supermimic/assets/motions \
  supermimic/assets/policies/baseline/policy.onnx \
  supermimic/assets/policies/backpack/policy.onnx \
  rl_lab_controller/cpp_prefix \
  rl_lab_controller/cyclonedds \
  rl_lab_controller/thirdparty \
  rl_lab_controller/python_sdk
printf 'Created %s\n' "$OUT"
