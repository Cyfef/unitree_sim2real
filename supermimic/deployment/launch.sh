#!/usr/bin/env bash
# Start the copied RL Lab FSM with a validated SuperMimic adapter.
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/config/env.sh"
NETWORK="$SUPERMIMIC_NETWORK"
SESSION=''
SUPPORTED=0
EXECUTE=0
while (($#)); do
  case "$1" in
    --session) SESSION="${2:?missing session}"; shift 2 ;;
    --network) NETWORK="${2:?missing network}"; shift 2 ;;
    --supported-grounded) SUPPORTED=1; shift ;;
    --execute) EXECUTE=1; shift ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
if (( !SUPPORTED || !EXECUTE )) || [[ -z "$SESSION" ]]; then
  echo 'Requires --session, --supported-grounded and --execute.' >&2
  exit 2
fi
if [[ -n "${SSH_CONNECTION:-}" || -n "${SSH_CLIENT:-}" ]]; then
  echo 'Run at the robot workstation with an onsite operator, not over SSH.' >&2
  exit 2
fi
"$PYTHON_BIN" "$ROOT/deployment/doctor.py" --network "$NETWORK" --require-link
SESSION="$(realpath -- "$SESSION")"
[[ -f "$SESSION/candidate.json" && -f "$SESSION/reference.json" ]] || {
  echo 'Session is incomplete.' >&2; exit 2;
}
export CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-$UNITREE_SIM2REAL_ROOT/rl_lab_controller/cyclonedds}"
export PYTHONPATH="$UNITREE_SIM2REAL_ROOT/rl_lab_controller/python_sdk${PYTHONPATH:+:$PYTHONPATH}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
mkdir -p "$ROOT/outputs/deployment"
exec 9>"$ROOT/outputs/deployment/.launch.lock"
flock -n 9 || { echo 'Another SuperMimic launch is active.' >&2; exit 1; }
echo 'Checking saved isolated simulation validation (these are not live robot results):'
"$PYTHON_BIN" "$ROOT/deployment/verify.py" --session "$SESSION" --check-only
"$PYTHON_BIN" "$UNITREE_SIM2REAL_ROOT/rl_lab_controller/g1_29dof_deployment/policy_preflight.py" \
  --network "$NETWORK" --reference "$SESSION/reference.json"
mkdir -p "$SESSION/real_logs"
recorder_pid=''
cleanup() {
  if [[ -n "$recorder_pid" ]] && kill -0 "$recorder_pid" 2>/dev/null; then
    sleep .7
    kill -TERM "$recorder_pid" 2>/dev/null || true
    wait "$recorder_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
setsid "$PYTHON_BIN" -u "$UNITREE_SIM2REAL_ROOT/rl_lab_controller/g1_29dof_deployment/policy_record.py" \
  --network "$NETWORK" --output "$SESSION/real_logs/telemetry.csv" \
  --rate 100 --duration 1800 >"$SESSION/real_logs/telemetry.log" 2>&1 &
recorder_pid=$!
sleep .5
kill -0 "$recorder_pid"
echo 'Controller starts Passive; automatic FixStand begins after 3 seconds.'
echo 'FixStand recovers nominal posture, waits for stability, then aligns to the first frame. Tracking stalls pause the path and time out.'
echo 'After FixStand is ready: R1+A enters static policy standby; fresh R1+X starts motion when ready.'
echo 'Motion ends in policy final-pose hold, not automatic disable. L2+B returns to damping; Ctrl+C shuts down.'
"$UNITREE_SIM2REAL_ROOT/rl_lab_controller/g1_29dof_deployment/build/g1_ctrl" --network "$NETWORK" \
  --config "$SESSION/candidate.json" 2>&1 | tee "$SESSION/real_logs/controller.log"
