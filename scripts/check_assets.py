#!/usr/bin/env python3
"""Verify the runtime asset bundle without importing robot SDK or connecting DDS."""
from pathlib import Path
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
manifest = json.loads((ROOT / 'asset_manifest.json').read_text())
errors = []
for rel, expected in manifest['sha256'].items():
    path = ROOT / rel
    if not path.is_file():
        errors.append(f'missing: {rel}')
        continue
    actual = hashlib.sha256(path.read_bytes()).hexdigest()
    if actual != expected:
        errors.append(f'changed: {rel}')
for rel in manifest['required_directories']:
    if not (ROOT / rel).is_dir(): errors.append(f'missing directory: {rel}')
if errors:
    raise SystemExit('Asset check failed:\n' + '\n'.join(errors[:30]))
print(f"Asset check passed: {len(manifest['sha256'])} files")
