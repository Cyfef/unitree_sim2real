"""Read-only preflight for SuperMimic simulation inputs."""
from __future__ import annotations

from pathlib import Path
import argparse
import json

BASE = Path(__file__).resolve().parent
TASK = BASE.parent
ROOT = TASK.parent
def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', choices=('baseline', 'backpack'), default='backpack')
    args = parser.parse_args()
    required = [
        ROOT / 'rl_lab_controller/g1_29dof_deployment/build_simulation/g1_ctrl',
        ROOT / 'models/g1_29dof_with_hand_rev_1_0.xml',
        ROOT / 'models/meshes/backpack_link.STL',
        TASK / 'assets/policies' / args.policy / 'policy.onnx',
        TASK / 'assets/policies' / args.policy / 'policy_meta.json',
        BASE / 'poses/standby.json',
        BASE / 'physics.json',
        ROOT / 'simulation_core/physics.py',
    ]
    missing = [str(path) for path in required if not path.is_file()]
    physics = json.loads((BASE / 'physics.json').read_text())
    if len(physics.get('options', {})) != 29 or len(physics.get('joints', {})) != 29:
        missing.append('simulation physics configuration is incomplete')
    clips = list((TASK / 'assets/motions').glob('*.csv'))
    if len(clips) != 40:
        missing.append(f'expected 40 motion CSVs, found {len(clips)}')
    print(json.dumps({'ready': not missing, 'policy': args.policy, 'clips': len(clips), 'missing': missing}, indent=2))
    if missing:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
