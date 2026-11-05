# SuperMimic assets

`manifest.json` lists the task's policy weights and motion CSVs with their hashes and bundle paths. It is checked together with the workspace's shared `../../asset_manifest.json` by `../../scripts/check_assets.py`.

`policies/` contains the baseline and backpack ONNX weights and their versioned metadata. `motions/` contains the 40 reference clips. Large ONNX and CSV files are supplied by the asset bundle; the manifest and metadata stay in Git.
