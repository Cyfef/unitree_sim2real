# SuperMimic simulation

This directory contains the SuperMimic simulation adapter. It runs the shared G1 hand model with a 1.9 kg backpack attached at runtime, using task-owned numerical settings in `physics.json`. The shared `simulation_core/physics.py` applies those settings. The simulation controller in `controller/` is built separately with simulation-only joint limits and requires a loopback-only network namespace.

The `baseline`, `backpack`, and `backpack_retrain` policy weights use the same model, motion files, fixed starting pose in `poses/standby.json`, and physics settings. The selected ONNX and matching metadata are recorded in each run. The deployment validation in `../deployment/` is a separate flow with its own physics settings; an onsite operator may explicitly skip it using `deploy.sh prepare --skip-simulation`, which records that choice.

From the workspace root, after installing assets and setting up the environment:

```bash
bash supermimic/simulation/build.sh
python3 supermimic/simulation/check_inputs.py --policy backpack
.venv/bin/python supermimic/simulation/run_batch.py --policy backpack --name batch_YYYYMMDD --workers 8
```

Choose `--policy baseline` or `--policy backpack_retrain` to run another policy. FixStand uses the recovery-template `kp` (identical to the official 29-DoF values) and the selected SuperMimic policy metadata `kd` by default, matching the earlier simulation combination. Use `--stand-gains original` for the recovery-template `kp` and `kd` used by onsite deployment, or `--stand-gains official` for the official FixStand `kp` and `kd` recorded in `official_fixstand_gains.json`. This selection affects only FixStand, not the selected policy gains or onsite deployment. `run_batch.py` evaluates all 40 bundled motion clips from their first frame and writes one result per clip. The last command runs the simulation; the first two only build and inspect inputs. Use a new batch name for each attempt under a policy.

To save policy-motion videos for a new 40-clip batch, add `--record-video` to `run_batch.py`. The physics runs first; after all clips finish, the batch renders one 640×480, 20 FPS MP4 at a time, without changing physics timing or pass/fail criteria. Videos are stored at `runs/<clip>/simulation/video.mp4`, with the captured pose trace in `video_states.npz`; `progress.json` and `results.json` include each clip's `video_status`. If a clip fails before entering motion, no video trace is available. Individual `run.py` invocations also accept `--record-video` and render after that run. Rendering requires `imageio-ffmpeg` from the task requirements and a working MuJoCo EGL renderer. If rendering is interrupted after the simulation has finished, run `.venv/bin/python supermimic/simulation/render_video.py --run <run-directory>` for a clip that has `video_states.npz` but no `video.mp4`. Existing batches without `video_states.npz` do not contain the poses needed for an exact video replay.

```bash
.venv/bin/python supermimic/simulation/run_batch.py \
  --policy backpack --name backpack_with_video --workers 3 --record-video
```

For the SuperMimic standby flow, the simulation bridge waits for the controller to report FixStand readiness before sending R1+A, then waits for confirmed policy entry before fading virtual support. R1+X is sent after the policy reports standby readiness. A missing readiness/entry event fails the run instead of releasing support on a timer. These automatic inputs and virtual forces exist only in simulation; onsite deployment still uses operator inputs and its existing safety configuration.

Results are stored under `../outputs/simulation/<policy>/batches/<name>/`: `manifest.json`, `progress.json`, `results.json`, `candidates/<clip>/`, `runs/<clip>/`, and `logs/`. Individual runs started with `run.py` go under `../outputs/simulation/<policy>/manual/<name>/`. Generated data is excluded from Git.
