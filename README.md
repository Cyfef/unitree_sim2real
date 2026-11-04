# Unitree G1 sim2real workspace

A movable G1-29DoF workspace with shared robot interfaces and task-specific adapters. The current task is [SuperMimic](supermimic/README.md). A future task can live beside `supermimic/` and reuse the shared SDK, model assets, controller foundation, and simulation dependencies.

## Layout

| Path | Purpose |
| --- | --- |
| `rl_lab_controller/` | Shared Unitree SDK2/DDS dependencies, G1 state and safety foundation, C++ controller build |
| `models/` | G1 model with Dex3 hands and meshes; simulation attaches the 1.9 kg backpack at runtime |
| `supermimic/controller/` | SuperMimic C++ policy state and entry point compiled into the shared controller |
| `supermimic/deployment_synced/` | Preparation, isolated same-controller simulation, validation, and launch |
| `supermimic/assets/policies/` | Two policy variants and their metadata |
| `supermimic/assets/motions/` | 40 G1 retargeted reference motions |
| `scripts/` | Environment setup and external asset-bundle tools |

Paths inside scripts are derived from their own file location. The checkout may be moved as a whole. Old prepared sessions are bound to the old path and must be prepared again after a move.

## First use

This Git repository contains code, documentation, model descriptions and small policy metadata. Large weights, motion CSVs, meshes, and third-party binary SDK dependencies are intentionally outside Git. The complete working directory already contains them. For a fresh clone, copy the separately distributed asset bundle and run:

```bash
bash scripts/install_asset_bundle.sh ./unitree_sim2real-assets.tar.gz
bash scripts/setup.sh
cd supermimic
bash deploy.sh check
```

See [environment setup](ENVIRONMENT.md) and [SuperMimic usage](supermimic/README.md). `check` is offline and does not communicate with the robot. Onsite `prepare` and `run` instructions are in the task README.

To create an asset bundle from a complete working directory:

```bash
bash scripts/make_asset_bundle.sh ./unitree_sim2real-assets.tar.gz
```

The archive is not committed to Git. Its contents are checked against `asset_manifest.json` after installation.

## Scope

The two bundled SuperMimic policies use the same 29 body joints but were trained against different inertial assumptions. Both deployment-flow simulations use the hands/backpack G1 model; neither test alone proves a policy safe on a physical robot. The real-robot launcher requires a newly prepared and isolated-simulation-validated session.
