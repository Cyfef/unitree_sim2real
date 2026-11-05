# SuperMimic simulation

This directory contains the SuperMimic simulation adapter. It runs the shared G1 hand model with a 1.9 kg backpack attached at runtime, using task-owned numerical settings in `physics.json`. The shared `simulation_core/physics.py` applies those settings. The simulation controller in `controller/` is built separately with simulation-only joint limits and requires a loopback-only network namespace.

Both `baseline` and `backpack` policy weights use the same model, motion files, fixed starting pose in `poses/standby.json`, and physics settings. The selected ONNX and matching metadata are recorded in each run. The deployment validation in `../deployment/` is a separate flow with its own physics settings and is required before a real-robot run.

From the workspace root, after installing assets and setting up the environment:

```bash
bash supermimic/simulation/build.sh
python3 supermimic/simulation/check_inputs.py --policy backpack
.venv/bin/python supermimic/simulation/run_batch.py --policy backpack --name batch_YYYYMMDD --workers 8
```

Choose `--policy baseline` to run the other policy. `run_batch.py` evaluates all 40 bundled motion clips from their first frame and writes one result per clip. The last command runs the simulation; the first two only build and inspect inputs. Use a new batch name for each attempt under a policy.

Results are stored under `../outputs/simulation/<policy>/batches/<name>/`: `manifest.json`, `progress.json`, `results.json`, `candidates/<clip>/`, `runs/<clip>/`, and `logs/`. Individual runs started with `run.py` go under `../outputs/simulation/<policy>/manual/<name>/`. Generated data is excluded from Git.
