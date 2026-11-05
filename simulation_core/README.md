# Shared simulation code

`physics.py` applies a task-owned MuJoCo JSON profile to a compiled model. It validates option names, array dimensions, finite values, and one-DoF joint settings. It contains no robot model, policy, motion data, or task-specific numbers.

A task keeps its own physics profile and model preparation in its folder, then calls `apply_profile(model, profile_path)`. SuperMimic uses this shared loader for its task simulation. Its 5 ms Euler settings and joint losses are stored in `supermimic/simulation/physics.json`.
