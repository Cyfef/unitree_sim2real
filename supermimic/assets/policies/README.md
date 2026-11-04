# Policy variants

`baseline/` and `backpack/` each contain `policy_meta.json` in Git and a large `policy.onnx` supplied by the asset bundle. Use the `--policy` option of `deploy.sh prepare` to select one. Do not replace an ONNX file without regenerating its metadata and the workspace asset manifest.
