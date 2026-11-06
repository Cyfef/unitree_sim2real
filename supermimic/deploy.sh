#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/config/env.sh"
usage() {
  cat <<'TXT'
Usage:
  bash deploy.sh check [--network enp12s0]
    Offline dependency/namespace checks; no robot queries or commands.
  bash deploy.sh prepare --policy baseline|backpack --clip NAME [--start-seconds 0] [--max-seconds 5 | --full-motion] --supported-grounded [--network NAME]
    ONSITE ONLY: release official control (motors may relax), capture current
    pose, prepare selected clip, verify in isolated simulation. Does not launch policy.
  bash deploy.sh run --session PATH --supported-grounded --execute [--network NAME]
    ONSITE ONLY: validated controller launch; automatic FixStand after ~3 seconds.
TXT
}
command="${1:-help}"
if (($#)); then shift; fi
case "$command" in
  check) exec "$PYTHON_BIN" "$ROOT/deployment/doctor.py" "$@" ;;
  run) exec bash "$ROOT/launch.sh" "$@" ;;
  help|--help|-h) usage; exit 0 ;;
  prepare) ;;
  *) usage >&2; exit 2 ;;
esac
POLICY=''; CLIP=''; START=0; SECONDS_TO_PLAY=5; FULL_MOTION=0; NETWORK="$SUPERMIMIC_NETWORK"; SUPPORTED=0
while (($#)); do
  case "$1" in
    --policy) POLICY="${2:?missing policy}"; shift 2 ;;
    --clip) CLIP="${2:?missing clip}"; shift 2 ;;
    --start-seconds) START="${2:?missing start}"; shift 2 ;;
    --max-seconds) SECONDS_TO_PLAY="${2:?missing duration}"; shift 2 ;;
    --full-motion) FULL_MOTION=1; shift ;;
    --network) NETWORK="${2:?missing network}"; shift 2 ;;
    --supported-grounded) SUPPORTED=1; shift ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
if [[ -n "${SSH_CONNECTION:-}" || -n "${SSH_CLIENT:-}" ]]; then
  echo 'Run prepare at the robot workstation with an onsite operator, not over SSH.' >&2; exit 2
fi
if (( !SUPPORTED )) || [[ -z "$CLIP" ]] || [[ "$POLICY" != baseline && "$POLICY" != backpack ]]; then
  echo 'prepare requires --policy baseline|backpack, --clip and --supported-grounded.' >&2; exit 2
fi
# Validate the selected motion before any robot release call. Reference entries are 20 ms apart.
"$PYTHON_BIN" - "$ROOT" "$CLIP" "$START" "$SECONDS_TO_PLAY" "$FULL_MOTION" <<'PYVALIDATE'
import sys, math
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from contract import load_clip
root, clip, start_arg, duration_arg, full_arg = sys.argv[1:]
if Path(clip).name != clip:
    raise SystemExit('Invalid clip name')
motion = load_clip(Path(root) / 'assets/motions' / f'{clip}.csv', 10)
total = (motion.frames - 1) * .02
if full_arg == '1':
    if total < 5 or total > 600:
        raise SystemExit('Full motion duration must be 5–600 seconds')
    print(f'Full motion selected: 0–{total:.2f}s; start and max seconds ignored')
else:
    start, duration = float(start_arg), float(duration_arg)
    if not math.isfinite(start) or not math.isfinite(duration) or start < 0 or duration < 5:
        raise SystemExit('Start must be >= 0; requested duration must be >= 5 seconds')
    i = round(start / .02)
    remaining = (motion.frames - 1 - i) * .02
    if remaining < 5 - 1e-8:
        raise SystemExit('Fewer than 5 seconds remain at the selected start')
    effective = min(duration, remaining)
    print(f'Motion window: {i * .02:.2f}–{i * .02 + effective:.2f}s'
          + (' (truncated at clip end)' if duration > remaining else ''))
PYVALIDATE
"$PYTHON_BIN" "$ROOT/deployment/doctor.py" --network "$NETWORK" --require-link
OUTPUT_ROOT="$ROOT/outputs/deployment"
mkdir -p "$OUTPUT_ROOT/sessions"
exec 9>"$OUTPUT_ROOT/.launch.lock"
flock -n 9 || { echo 'Another deployment session is active.' >&2; exit 1; }
SESSION="$(mktemp -d "$OUTPUT_ROOT/sessions/onsite-unitree-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
trap 'echo "Stopped. Logs: $SESSION. Keep robot supported; inspect the error before retrying." >&2' ERR
mkdir -p "$SESSION/logs"
echo "Session: $SESSION"
echo 'Releasing official control and disabling motors; full hoist support is required.'
"$PYTHON_BIN" "$UNITREE_SIM2REAL_ROOT/rl_lab_controller/g1_29dof_deployment/policy_release.py" --network "$NETWORK" --supported-grounded --output "$SESSION/release.json" 2>&1 | tee "$SESSION/logs/release.log"
"$PYTHON_BIN" "$ROOT/capture_reference.py" --network "$NETWORK" --output "$SESSION/reference.json" 2>&1 | tee "$SESSION/logs/capture.log"
PREPARE_ARGS=(--policy "$POLICY" --clip "$CLIP" --reference "$SESSION/reference.json" --output-dir "$SESSION/candidate")
if (( FULL_MOTION )); then
  PREPARE_ARGS+=(--full-motion)
else
  PREPARE_ARGS+=(--start-seconds "$START" --max-seconds "$SECONDS_TO_PLAY")
fi
"$PYTHON_BIN" "$ROOT/deployment/prepare.py" "${PREPARE_ARGS[@]}" 2>&1 | tee "$SESSION/logs/prepare.log"
"$PYTHON_BIN" "$ROOT/deployment/verify.py" --session "$SESSION/candidate" 2>&1 | tee "$SESSION/logs/verify.log"
echo 'Prepared and verified. Policy has NOT been launched. Launch separately onsite:'
printf 'bash %q run --session %q --network %q --supported-grounded --execute\n' "$ROOT/deploy.sh" "$SESSION/candidate" "$NETWORK"
