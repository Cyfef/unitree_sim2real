"""Export the bundled checkpoint for the copied RL Lab ONNX inference backend.

Offline only: no SDK import and no DDS initialization.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import onnxruntime as ort
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from contract import JOINT_NAMES, default_q, gains_and_limits, load_policy


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--checkpoint", type=Path, required=True)
    p.add_argument("--output-dir", type=Path, required=True)
    args = p.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    policy, hist, future = load_policy(args.checkpoint)
    if (hist, future) != (10, 10):
        raise ValueError(f"expected (hist, future)=(10,10), got {(hist, future)}")
    obs = torch.zeros(1, hist, 99)
    goal = torch.zeros(1, 73 * future)
    output = args.output_dir / "policy.onnx"
    torch.onnx.export(policy, (obs, goal), output, opset_version=17,
                      input_names=["obs_hist", "goal"], output_names=["action"],
                      do_constant_folding=True, dynamo=False)
    session = ort.InferenceSession(str(output), providers=["CPUExecutionProvider"])
    rng = np.random.default_rng(2026)
    worst = 0.0
    for _ in range(8):
        x = rng.normal(size=(1, hist, 99)).astype(np.float32)
        g = rng.normal(size=(1, 73 * future)).astype(np.float32)
        with torch.inference_mode():
            expected = policy(torch.from_numpy(x), torch.from_numpy(g)).numpy()
        actual = session.run(None, {"obs_hist": x, "goal": g})[0]
        worst = max(worst, float(np.max(np.abs(actual - expected))))
    if worst > 1e-4:
        output.unlink(missing_ok=True)
        raise RuntimeError(f"ONNX/PyTorch parity failed: max abs difference {worst}")
    kp, kd, scale, _, _ = gains_and_limits()
    meta = {
        "hist": hist, "goal_frames": future, "obs_dim": 99, "goal_dim": 73 * future,
        "joint_names": JOINT_NAMES, "default_q": default_q().tolist(),
        "kp": kp.astype(float).tolist(), "kd": kd.astype(float).tolist(),
        "action_scale": scale.astype(float).tolist(),
        "checkpoint_sha256": digest(args.checkpoint), "onnx_sha256": digest(output),
        "max_onnx_pytorch_abs_diff": worst,
    }
    (args.output_dir / "policy_meta.json").write_text(json.dumps(meta, indent=2) + "\n")
    print(json.dumps({"onnx": str(output), "max_abs_diff": worst, "hist": hist,
                      "goal_frames": future}))


if __name__ == "__main__":
    main()
