# Environment setup

Run setup from the workspace root. The current workspace targets Ubuntu 24.04 x86_64 with Python 3.12, CMake 3.24 or newer, a C++17 compiler, `ip`, `unshare`, and `sudo`. The SDK, CycloneDDS, ONNX Runtime C++ libraries, and model meshes are distributed in the asset bundle.

## Configuration layers

| Layer | Files | Responsibility |
| --- | --- | --- |
| Workspace | `requirements.txt`, `scripts/env.sh`, `asset_manifest.json` | Shared Python packages, SDK/runtime paths, and shared assets |
| Task | `<task>/requirements.txt`, `<task>/config/env.sh`, `<task>/assets/manifest.json` | Policy tooling, task defaults, and task assets |

The workspace creates one `.venv/` and one shared controller build. `scripts/setup.sh <task>` installs shared requirements first, then the selected task's requirements, and builds the shared G1 controller with that task's C++ adapter. The default task is `supermimic`. A new task supplies its own controller adapter and asset manifest; it does not copy the SDK or model directory.

## First setup

```bash
bash scripts/install_asset_bundle.sh ./unitree_sim2real-assets.tar.gz supermimic
bash scripts/setup.sh supermimic
python3 scripts/check_assets.py supermimic
cd supermimic && bash deploy.sh check
```

The asset check reads the shared `asset_manifest.json` and the task's `assets/manifest.json`. The offline deployment check imports dependencies, loads the model, and checks network isolation without initializing DDS or sending robot commands. The simulation controller is a separate build; use `bash supermimic/simulation/build.sh` when preparing simulation.

To create an asset bundle from a complete workspace:

```bash
bash scripts/make_asset_bundle.sh ./unitree_sim2real-assets.tar.gz supermimic
```

The root script packs the paths listed by the shared and task manifests. The archive is excluded from Git. A fresh clone needs this bundle before setup; a copied workspace should rerun setup on its destination machine to rebuild the virtual environment and C++ binaries.

## Runtime environment

`scripts/env.sh` resolves the workspace's SDK, CycloneDDS, ONNX Runtime C++ and `.venv` paths. `supermimic/config/env.sh` sources it and adds the SuperMimic network default and optional Python override. The default robot interface is `enp12s0`; set `SUPERMIMIC_NETWORK` or pass `--network` onsite if it differs. `SUPERMIMIC_PYTHON` overrides the task's Python executable; `UNITREE_SIM2REAL_PYTHON` is the shared default.

The simulation uses a network namespace containing only `lo`. Where unprivileged namespaces are unavailable, the isolation helper uses `sudo` to create that namespace and drops privileges before running the test. Do not run the whole deployment script with `sudo`.

Git excludes `.venv/`, controller builds, SDK binaries, meshes, policy ONNX files, motion CSVs, generated outputs and logs. Code plus the asset bundle forms the complete working tree.
