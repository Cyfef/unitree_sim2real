#!/usr/bin/env bash
# Shared SDK, Python environment, and native runtime paths for every task.
export UNITREE_SIM2REAL_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHON_BIN="${UNITREE_SIM2REAL_PYTHON:-$UNITREE_SIM2REAL_ROOT/.venv/bin/python}"
export CYCLONEDDS_HOME="$UNITREE_SIM2REAL_ROOT/rl_lab_controller/cyclonedds"
export PYTHONPATH="$UNITREE_SIM2REAL_ROOT/rl_lab_controller/python_sdk${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$UNITREE_SIM2REAL_ROOT/rl_lab_controller/cpp_prefix/lib:$UNITREE_SIM2REAL_ROOT/rl_lab_controller/cpp_prefix/usr/lib/x86_64-linux-gnu:$UNITREE_SIM2REAL_ROOT/rl_lab_controller/thirdparty/onnxruntime-linux-x64-1.22.0/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
