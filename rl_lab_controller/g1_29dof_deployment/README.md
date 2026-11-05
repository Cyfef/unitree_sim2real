# Shared G1 controller foundation

This directory contains the shared G1 controller build, motor command path, FSM, FixStand, safety guard, and robot preflight/recording helpers. It does not contain the SuperMimic policy state or entry point. The task-specific C++ files live in `../../supermimic/controller/`.

Build a task adapter from the repository root:

```bash
cmake -S rl_lab_controller/g1_29dof_deployment -B rl_lab_controller/g1_29dof_deployment/build -DCMAKE_BUILD_TYPE=Release -DTASK_ADAPTER_DIR="$PWD/supermimic/controller"
cmake --build rl_lab_controller/g1_29dof_deployment/build -j2 --target g1_ctrl
```

The resulting `build/g1_ctrl` is a task-specific binary assembled from this shared foundation and the selected adapter. The SuperMimic launcher is `supermimic/deploy.sh`; its candidate and offline validation must be regenerated after rebuilding. Do not use the old copied binary or old sessions to authorize hardware.
