# Unitree G1 sim2real workspace

A movable G1-29DoF workspace with shared robot interfaces and task-specific adapters. The current task is [SuperMimic](supermimic/README.md). A future task can live beside `supermimic/` and reuse the shared SDK, model assets, controller foundation, and simulation dependencies.

## Layout

| Path | Purpose |
| --- | --- |
| `rl_lab_controller/` | Shared Unitree SDK2/DDS dependencies, G1 state and safety foundation, C++ controller build |
| `models/` | Shared G1 model files, meshes, and preserved Unitree documentation |
| `simulation_core/` | Shared MuJoCo physics-profile application and validation |
| `supermimic/controller/` | SuperMimic C++ policy state and entry point compiled into the shared controller |
| `supermimic/simulation/` | Task simulation adapter, physics profile, and batch runner |
| `supermimic/deployment/` | Onsite preparation, isolated deployment validation, and launch |
| `supermimic/assets/policies/` | Two policy variants and their metadata |
| `supermimic/assets/motions/` | 40 G1 retargeted reference motions |
| `scripts/`, `requirements.txt`, `asset_manifest.json` | Shared environment setup, dependencies, and asset definitions |
| `supermimic/config/`, `supermimic/requirements.txt`, `supermimic/assets/manifest.json` | SuperMimic environment, controller configuration, dependencies, and task assets |

## Repository boundary

`rl_lab_controller/` owns the G1/DDS transport, motor command path, FSM, FixStand, and shared safety foundation. `models/` stores the selected shared robot descriptions and Unitree-provided documentation. Its official `README.md` and `motor_info.md` are preserved; an older official README copy is kept as `README_unitree_legacy.md`. The official catalog describes more models than the subset stored in this workspace; task-specific model choices belong in task code or configuration. `simulation_core/` owns reusable simulation configuration logic; each task owns its numerical physics profile and robot/policy adapters. `scripts/` owns workspace setup and asset checks. A task folder owns its policy adapter, observation/action contract, policy and motion data, recovery template, simulations, and deployment workflow. Do not copy the shared SDK, controller sources, models, or Python environment into a task folder.

Each task adapter provides `controller/main.cpp`, `controller/include/`, and `controller/src/*.cpp`. Build the shared controller with `-DTASK_ADAPTER_DIR=<absolute task/controller path>`; the task supplies its state registration and entry point. A new task adds its own folder and setup/launch instructions without editing the shared motor command implementation. Run `bash scripts/setup.sh <task-folder>` to build its adapter; the default task is `supermimic`. Candidate validation binds the exact controller binary and task assets, so rebuilds, moves and task changes require a new candidate and isolated validation. The outer `asset_manifest.json` describes shared assets; each task provides its own `assets/manifest.json` and deployment validation. The shared scripts combine these two layers.

Paths inside scripts are derived from their own file location. The checkout may be moved as a whole. Old prepared sessions are bound to the old path and must be prepared again after a move.

## First use

This Git repository contains code, documentation, model descriptions and small policy metadata. Large weights, motion CSVs, meshes, and third-party binary SDK dependencies are intentionally outside Git. The complete working directory already contains them. For a fresh clone, copy the separately distributed asset bundle and run:

```bash
bash scripts/install_asset_bundle.sh ./unitree_sim2real-assets.tar.gz supermimic
bash scripts/setup.sh supermimic
cd supermimic
bash deploy.sh check
```

See [environment setup](ENVIRONMENT.md) and [SuperMimic usage](supermimic/README.md). `check` is offline and does not communicate with the robot. Onsite `prepare` and `run` instructions are in the task README.

To create an asset bundle from a complete working directory:

```bash
bash scripts/make_asset_bundle.sh ./unitree_sim2real-assets.tar.gz supermimic
```

The archive is not committed to Git. Its contents are checked against the shared `asset_manifest.json` and `supermimic/assets/manifest.json` after installation.

## Scope

SuperMimic supports the baseline and backpack policies in its simulation and onsite deployment flows. Onsite deployment candidates select a policy with `--policy baseline|backpack`. The onsite deployment gate retains its separate isolated validation. Simulation alone does not prove physical safety; the launcher requires a newly prepared, validated candidate.
