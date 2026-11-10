# SuperMimic policies

`baseline/` contains the earlier onsite deployment policy from the old repository's `assets/policy/`. `backpack/` contains the checkpoint-132000 policy. `backpack_retrain/` contains the retrained checkpoint-014000 policy. All three have the same observation/action layout, joint order, action scale, default positions and policy gains; their weights and export provenance differ.

The SuperMimic simulator and onsite `deploy.sh prepare` select a policy with `--policy baseline|backpack|backpack_retrain` and bind its ONNX and matching `policy_meta.json` into a new candidate. The metadata is versioned; the large ONNX weights are supplied by the workspace asset bundle. A new deployment policy choice requires a fresh candidate and either isolated validation or an explicit recorded simulation skip. Simulation keeps the same robot model and physics configuration for all policy choices.
