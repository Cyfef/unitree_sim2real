#!/usr/bin/env python3
"""Create a synthetic neutral reference for isolated simulation only."""
from pathlib import Path
import json
import sys

TASK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TASK))
from contract import default_q

output = Path(sys.argv[1])
output.write_text(json.dumps({
    'q': default_q().tolist(),
    'hands': {'left': [0.0] * 7, 'right': [0.0] * 7},
    'pelvis_quaternion': [1.0, 0.0, 0.0, 0.0],
    'synthetic_simulation_only': True,
}, indent=2) + '\n')
