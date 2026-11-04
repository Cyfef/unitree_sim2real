#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ARCHIVE="${1:?give the asset bundle .tar.gz path}"
# Check paths before extracting an archive supplied by the operator.
tar -tzf "$ARCHIVE" | python3 -c 'import sys,pathlib; names=[s.strip() for s in sys.stdin]; assert all(s and not pathlib.PurePosixPath(s).is_absolute() and ".." not in pathlib.PurePosixPath(s).parts for s in names), "unsafe archive path"'
tar -C "$ROOT" -xzf "$ARCHIVE"
python3 "$ROOT/scripts/check_assets.py"
