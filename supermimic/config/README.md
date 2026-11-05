# SuperMimic configuration

`env.sh` adds SuperMimic's network interface default and optional Python override after loading the shared workspace environment from `../../scripts/env.sh`. Set `SUPERMIMIC_NETWORK` or pass `--network` at the robot workstation. `SUPERMIMIC_PYTHON` can select a different Python executable for this task.

`recovery_template.json` is the task's posture and controller template used when preparing a motion. The simulation physics profile is in `../simulation/physics.json`; it is task-owned but separate from the deployment template. Shared SDK paths and Python packages remain in the outer workspace.
