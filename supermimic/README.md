# SuperMimic G1 simulation and deployment

This task owns the SuperMimic policy adapter and its two required flows. The parent workspace supplies the G1 model, SDK/DDS, shared controller foundation, and reusable simulation-profile loader. All commands below run from `supermimic/`.

Both simulation and onsite deployment can select the backpack policy in `assets/policies/backpack/` or the baseline policy in `assets/policies/baseline/`. Each ONNX is paired with its own metadata; both preserve the same observation/action and motor-gain contract. The G1 model is `../models/g1_29dof_with_hand_rev_1_0.xml`; simulation attaches an assumed 1.9 kg backpack at runtime.

## Task configuration

Shared Python packages, SDK paths, model files, and setup scripts live at the workspace root. SuperMimic adds policy tooling in `requirements.txt`, runtime defaults and recovery settings in `config/`, and policy/motion asset hashes in `assets/manifest.json`. The outer setup script installs both dependency layers and builds this task's controller adapter. See [environment setup](../ENVIRONMENT.md) and [task configuration](config/README.md).

## Simulate SuperMimic

The [simulation adapter](simulation/README.md) evaluates the 40 motion clips with either policy. It uses a fixed starting pose, 5 ms Euler physics, one physics step per DDS state, policy gains during stand, and a simulation-only controller and safety profile. Numerical settings are in `simulation/physics.json` and are applied by the shared `simulation_core/` module. See the simulation README for build, input-check, and batch-run commands.

## Prepare and deploy onsite

The `deploy.sh` chain performs onsite preparation: release the official controller and disable motors, capture a fresh robot pose, prepare a bounded candidate, validate the same C++ controller in isolated nominal and arm-load simulations, then print a separate run command. It binds the selected ONNX and its matching metadata to each candidate. The validation simulation is a deployment gate with 1 ms implicitfast physics substeps; it is separate from the task simulation above.

After installing the asset bundle and running `bash ../scripts/setup.sh`, check dependencies offline:

```bash
bash deploy.sh check
```

At the robot workstation, with the robot fully supported, feet grounded, and an operator at the emergency stop:

```bash
bash deploy.sh prepare --policy backpack --clip walk1_subject2 \
  --start-seconds 0 --max-seconds 5 --network enp12s0 --supported-grounded
```

Choose `--policy baseline` for the earlier deployment weight or `--policy backpack` for checkpoint 132000. This selects the paired ONNX and metadata only; the model, controller, motor command path, safety gates and deployment validation settings stay the same. `prepare` refuses SSH and does not launch the policy. It prints a new `outputs/deployment/sessions/.../candidate` path and the run command after validation. Run that exact command onsite, for example:

```bash
bash deploy.sh run --session ./outputs/deployment/sessions/onsite-unitree-.../candidate \
  --network enp12s0 --supported-grounded --execute
```

The launcher checks the saved validation and current hashes, checks live robot state, then starts the shared controller. FixStand starts after about 3 seconds; R1+A enters standby and R1+X starts the motion. L2+B enters Passive damping; Ctrl+C disables commands. Keep physical support throughout. Do not reuse a candidate after moving or rebuilding the checkout, changing the policy, or changing the model; prepare and validate again.

## Generated data

All generated SuperMimic data is under `outputs/` and is excluded from Git:

- `outputs/simulation/<policy>/batches/<name>/` contains one motion batch: results and manifests at the batch root, prepared clips in `candidates/`, per-clip simulation traces in `runs/`, and process logs in `logs/`. Single-clip runs go under `<policy>/manual/<name>/`.
- `outputs/deployment/sessions/<session>/` contains one onsite preparation and run: `logs/` for preparation and validation commands, `candidate/` for bound assets and isolated validation results, and `candidate/real_logs/` for actual robot telemetry and controller traces.
- `outputs/deployment/legacy_sessions/` preserves earlier offline records moved from the old `sessions/` location. Their saved absolute paths and validation hashes are stale; create a new candidate for any future deployment.

## Ownership

`controller/` contains the SuperMimic policy state and C++ entry point. `config/recovery_template.json`, the policy, motions, fixed simulation pose, and simulation physics profile are task-specific. Shared SDK, motor command path, FSM foundation, model files, and physics-profile application live in `../rl_lab_controller/`, `../models/`, and `../simulation_core/`. Future tasks add their own folder and adapters rather than copying those shared foundations.
