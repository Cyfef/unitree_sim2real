#!/usr/bin/env bash
# SuperMimic-specific environment layered on the shared workspace runtime.
export SUPERMIMIC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SUPERMIMIC_ROOT/../scripts/env.sh"
export PYTHON_BIN="${SUPERMIMIC_PYTHON:-$PYTHON_BIN}"
export SUPERMIMIC_NETWORK="${SUPERMIMIC_NETWORK:-enp12s0}"
