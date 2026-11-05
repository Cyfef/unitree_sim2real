#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:?give an output .tar.gz path}"
TASK_NAME="${2:-supermimic}"
python3 "$ROOT/scripts/check_assets.py" "$TASK_NAME"
BUNDLE_PATH_TEXT="$(python3 "$ROOT/scripts/check_assets.py" "$TASK_NAME" --bundle-paths)"
[[ -n "$BUNDLE_PATH_TEXT" ]] || { echo "No assets to pack for $TASK_NAME" >&2; exit 1; }
mapfile -t BUNDLE_PATHS <<< "$BUNDLE_PATH_TEXT"
mkdir -p "$(dirname -- "$OUT")"
tar -C "$ROOT" -czf "$OUT" "${BUNDLE_PATHS[@]}"
printf 'Created %s for %s\n' "$OUT" "$TASK_NAME"
