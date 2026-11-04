#!/usr/bin/env bash
# Simulate the same C++ controller and G1 hands/backpack model in a loopback-only namespace.
set -Eeuo pipefail
TASK="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$TASK/deployment_synced/env.sh"
POLICY="${1:?policy: baseline or backpack}"
CLIP="${2:?motion clip name}"
SECONDS="${3:-1}"
[[ "$POLICY" == baseline || "$POLICY" == backpack ]] || { echo 'Unknown policy' >&2; exit 2; }
mkdir -p "$TASK/sessions"
SESSION="$(mktemp -d "$TASK/sessions/offline-${POLICY}-${CLIP}-XXXXXX")"
"$PYTHON_BIN" "$TASK/simulation/make_reference.py" "$SESSION/reference.json"
"$PYTHON_BIN" "$TASK/deployment_synced/prepare.py" --policy "$POLICY" --clip "$CLIP" \
  --start-seconds 0 --max-seconds "$SECONDS" --reference "$SESSION/reference.json" \
  --output-dir "$SESSION/candidate" 2>&1 | tee "$SESSION/prepare.log"
"$PYTHON_BIN" "$TASK/deployment_synced/verify.py" --session "$SESSION/candidate" 2>&1 | tee "$SESSION/verify.log"
printf 'Simulation-only results: %s\n' "$SESSION"
