# SuperMimic G1 deployment and simulation

Run commands in this README from `supermimic/`. The robot is a 29-DoF G1. The task uses 40 retargeted G1 motion CSVs. The shared controller, SDK dependencies, and model files are in the parent workspace; relative symlinks under this task preserve the existing tested interfaces.

## Choose a policy

| `--policy` | Source training model | Weight | Deployment-flow simulation model |
| --- | --- | --- | --- |
| `baseline` | Original G1 29-DoF model, rubber-hand proxies, no backpack payload randomization | `assets/policies/baseline/policy.onnx` | G1 with Dex3 hands and 1.9 kg backpack |
| `backpack` | G1 29-DoF with randomized 1.9 kg backpack and hand inertial properties; geometry unchanged during training | `assets/policies/backpack/policy.onnx` | The same hands/backpack deployment model |

Both variants control the same 29 body joints and use the same observation/goal layout, gains and 40 reference motions. Their ONNX files are different; each has its own metadata and SHA-256. The `backpack` policy was exported from checkpoint 132000. The preparer records the policy choice in the candidate and the verifier binds the selected ONNX and metadata by hash, so a later policy swap invalidates that candidate.

The simulation model is `../models/g1_29dof_with_hand_rev_1_0.xml` with the backpack attached at runtime. The backpack mass is assumed to be 1.9 kg; the real Thor unit's mass and shape have not been measured. The simulator fixes fingers to the captured pose and controls only body joints. The deployment-flow validation uses the same controller binary as the onsite launcher, with isolated nominal and arm-load stress simulations. Physics step is 1 ms, DDS state step 5 ms and policy step 20 ms. This is separate from the native 5 ms training/sim2sim test.

## Offline setup and check

From the workspace root, run `bash scripts/setup.sh` after installing the asset bundle as described in [environment setup](../ENVIRONMENT.md). Then:

```bash
cd supermimic
bash deploy.sh check
```

This checks both policy files, model, motions, dependencies and isolated simulation namespace. It does not query the robot.

## Isolated simulation

Run `bash simulation/run.sh backpack walk1_subject2 1` or select `baseline`. This uses a synthetic starting pose and never authorizes hardware. See [simulation usage](simulation/README.md).

## Prepare and validate a motion onsite

Run at the robot workstation's local terminal with the robot fully supported, both feet grounded, and an operator at the emergency stop. `prepare` releases the factory controller and disables motors before capturing the current pose. It then generates a bounded candidate and validates it in two isolated simulations; it does **not** launch the policy. The script refuses an SSH session.

```bash
bash deploy.sh prepare --policy backpack --clip walk1_subject2 \
  --start-seconds 0 --max-seconds 5 --network enp12s0 --supported-grounded
```

To use the original policy, change `backpack` to `baseline`. The program prints a new `sessions/.../candidate` path and, after validation succeeds, the exact `run` command for that candidate. Never reuse a candidate from a different checkout or move; its absolute runtime bindings and validation hashes are tied to the checkout where it was prepared. You can create a simulation-only candidate from a fresh captured reference using `deployment_synced/prepare.py --full-clip-simulation`; this cannot authorize hardware launch.

## Launch a validated candidate onsite

Use the command printed by `prepare`, for example:

```bash
bash deploy.sh run --session ./sessions/onsite-unitree-.../candidate \
  --network enp12s0 --supported-grounded --execute
```

The launcher checks the saved validation and current file hashes, checks the physical robot state, then starts the shared controller. FixStand starts automatically after about 3 seconds. Follow the terminal prompts for R1+A standby and R1+X motion. L2+B enters Passive damping; Ctrl+C sends disabled commands. The operator must provide physical support throughout. Onsite traces are written into that candidate's `real_logs/` directory.

## Policy assets and portability

Large ONNX files, motion CSVs, meshes, and SDK binary dependencies are omitted from Git and supplied by the workspace asset bundle. `policy_meta.json` for each variant and `asset_manifest.json` are versioned. To replace a policy, export it using `tools/export_policy.py`, update the manifest, and prepare a fresh session. The controller must not be pointed at an unverified weight by editing a candidate JSON.

The code in `controller/` implements the SuperMimic policy state; the shared SDK/DDS and G1 controller foundation live in `../rl_lab_controller/`. Future tasks can add their own task folder and controller adapter without copying SuperMimic's motions or policy.
