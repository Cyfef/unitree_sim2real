# SuperMimic policies

`baseline/` contains the earlier onsite deployment policy from the old repository's `assets/policy/`. `backpack/` contains the checkpoint-132000 policy. Both have the same observation/action layout, joint order, action scale, default positions and policy gains; their weights and export provenance differ.

The SuperMimic simulator and onsite `deploy.sh prepare` both select either policy with `--policy baseline|backpack` and bind its ONNX and matching `policy_meta.json` into a new candidate. The metadata is versioned; the large ONNX weights are supplied by the workspace asset bundle. A new deployment policy choice requires a fresh candidate and isolated validation. Simulation keeps the same robot model and physics configuration for both policy choices.
