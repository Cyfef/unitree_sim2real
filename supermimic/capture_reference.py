"""Capture a fresh, read-only G1 startup reference using the validated lab preflight."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

LAB = Path(__file__).resolve().parent / "rl_lab_controller/g1_29dof"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", default="eno1")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists")
    sys.path.insert(0, str(LAB))
    from policy_preflight import inspect_robot  # read-only, no LowCmd publisher

    reference = inspect_robot(args.network)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as file:
        json.dump(reference, file, indent=2)
        file.write("\n")
    print(f"Saved read-only reference: {args.output}")


if __name__ == "__main__":
    main()
