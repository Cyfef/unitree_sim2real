#!/usr/bin/env python3
"""Run every bundled motion through the isolated SuperMimic controller."""
import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

TASK = Path(__file__).resolve().parents[1]

def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1048576), b""):
            h.update(block)
    return h.hexdigest()

def save(path, data):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2) + "\n")
    temporary.replace(path)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", choices=("baseline", "backpack"), required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    motions = sorted((TASK / "assets/motions").glob("*.csv"))
    if len(motions) != 40:
        parser.error(f"expected 40 motion CSVs, found {len(motions)}")
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=False)
    policy = TASK / "assets/policies" / args.policy / "policy.onnx"
    model = TASK.parent / "models/g1_29dof_with_hand_rev_1_0.xml"
    if not policy.is_file() or not model.is_file():
        parser.error("policy or G1 model missing")
    summary = dict(policy=args.policy, policy_sha256=sha(policy), model=str(model.relative_to(TASK.parent)),
                   model_sha256=sha(model), hands=True, backpack_mass_kg=1.9,
                   full_clip=True, physical_robot_run=False, started_at=datetime.now(timezone.utc).isoformat(),
                   total=len(motions), completed=0, results=[])
    save(output / "summary.json", summary)
    for index, motion in enumerate(motions, 1):
        clip = motion.stem
        log = output / f"{index:02d}-{clip}.log"
        print(f"[{index:02d}/{len(motions)}] {clip}: full clip", flush=True)
        with log.open("w") as stream:
            code = subprocess.run(["bash", str(TASK / "simulation/run.sh"), args.policy, clip, "full"],
                                  cwd=TASK, stdout=stream, stderr=subprocess.STDOUT).returncode
        result = dict(index=index, clip=clip, frames=sum(1 for _ in motion.open()),
                      duration_seconds=round((sum(1 for _ in motion.open())-1) * .02, 3),
                      passed=code==0, returncode=code, log=log.name,
                      finished_at=datetime.now(timezone.utc).isoformat())
        summary["results"].append(result)
        summary["completed"] = index
        save(output / "summary.json", summary)
        print(f"[{index:02d}/{len(motions)}] {clip}: {'PASS' if code == 0 else 'FAIL'}", flush=True)
    summary["finished_at"] = datetime.now(timezone.utc).isoformat()
    summary["passed"] = sum(r["passed"] for r in summary["results"])
    save(output / "summary.json", summary)
    print(f"Finished: {summary['passed']}/{len(motions)} passed. Results: {output}", flush=True)

if __name__ == "__main__":
    main()
