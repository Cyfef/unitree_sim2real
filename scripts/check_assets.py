#!/usr/bin/env python3
"""Verify shared and selected task assets without importing robot SDK or DDS."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re

ROOT = Path(__file__).resolve().parents[1]


def safe_path(value: str) -> Path:
    rel = PurePosixPath(value)
    if not value or rel.is_absolute() or '..' in rel.parts or str(rel) != value:
        raise ValueError(f'Invalid asset path: {value!r}')
    return ROOT / value


def manifest(path: Path) -> dict:
    data = json.loads(path.read_text())
    if data.get('format') != 1:
        raise ValueError(f'Unsupported asset manifest: {path}')
    return data


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('task', nargs='?', default='supermimic')
    parser.add_argument('--bundle-paths', action='store_true', help='print paths to pack after validation')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.task):
        parser.error('task must be a directory name')
    manifests = [manifest(ROOT / 'asset_manifest.json'),
                 manifest(ROOT / args.task / 'assets/manifest.json')]
    expected = {}
    for item in manifests:
        for rel, digest in item['sha256'].items():
            safe_path(rel)
            if rel in expected:
                raise ValueError(f'Duplicate asset: {rel}')
            expected[rel] = digest
    errors = []
    for rel, digest in expected.items():
        path = safe_path(rel)
        if not path.is_file():
            errors.append(f'missing: {rel}')
            continue
        actual = hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                actual.update(chunk)
        if actual.hexdigest() != digest:
            errors.append(f'changed: {rel}')
    for item in manifests:
        for rel in item['required_directories']:
            if not safe_path(rel).is_dir():
                errors.append(f'missing directory: {rel}')
    if errors:
        raise SystemExit('Asset check failed:\n' + '\n'.join(errors[:30]))
    if args.bundle_paths:
        for item in manifests:
            for rel in item['bundle_paths']:
                if not safe_path(rel).exists():
                    raise FileNotFoundError(rel)
                print(rel)
    else:
        print(f'Asset check passed: {len(expected)} files ({args.task})')


if __name__ == '__main__':
    main()
