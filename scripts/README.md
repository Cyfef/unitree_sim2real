# Shared workspace scripts

These scripts operate at the workspace level. Task-specific settings live in each task folder.

| Script | Purpose |
| --- | --- |
| `env.sh`, `runtime_env.py` | Export shared SDK, library, and Python subprocess environment paths |
| `setup.sh <task>` | Create `.venv`, install shared then task dependencies, and build the selected controller adapter |
| `check_assets.py <task>` | Verify shared and task asset manifests without robot access |
| `make_asset_bundle.sh <archive> [task]` | Package assets listed by the shared and task manifests |
| `install_asset_bundle.sh <archive> [task]` | Install an asset bundle and verify it |

The default task is `supermimic`. The shared manifest is `../asset_manifest.json`; a task provides `<task>/assets/manifest.json` with its own hashes, required directories, and bundle paths. The script paths are derived from their own location so the workspace can be moved.
