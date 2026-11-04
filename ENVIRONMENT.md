# Environment setup

Run all commands from this repository root. This workspace targets Ubuntu 24.04 on x86_64 with Python 3.12, CMake, a C++17 compiler, `ip`, `unshare`, and `sudo`. The bundled C++ dependencies and Python SDK source are supplied by the separate asset bundle. Python dependencies are pinned in `supermimic/requirements.txt`.

1. Install the large asset bundle with `bash scripts/install_asset_bundle.sh ./unitree_sim2real-assets.tar.gz`. The asset check verifies model meshes, both policy ONNX files, the motion CSVs, and core SDK files.
2. Run `bash scripts/setup.sh`. This creates or updates `.venv`, installs Python packages, and builds `rl_lab_controller/g1_29dof_synced/build/g1_ctrl` from the code in this checkout.
3. Run `cd supermimic && bash deploy.sh check`. The offline check validates imports, libraries, model loading, motions, policies, and network namespace isolation. It does not initialize DDS or send robot commands.

If a Python environment from another machine was copied with the directory, rerun `bash scripts/setup.sh` on the destination. The source tree uses relative path discovery; a compiled binary and Python environment may need rebuilding for a different OS or architecture.

The controller and simulation use the in-tree CycloneDDS, Unitree SDK2, ONNX Runtime and C++ dependency bundle. The default physical network interface is `enp12s0`; set `SUPERMIMIC_NETWORK` or pass `--network` onsite if it differs. The isolated simulation uses a network namespace containing only `lo`; on systems without unprivileged namespaces, the existing isolation helper uses `sudo` to create that namespace and drops privileges before running the test. Do not run the entire deployment script with `sudo`.

Git deliberately excludes `.venv/`, C++ build outputs, SDK binary packages, meshes, policy ONNX files, motion CSVs, generated sessions and logs. The working directory and the Git repository therefore serve different purposes: copy the full folder for immediate use on a compatible workstation, or pair a fresh clone with the separately distributed asset bundle and run setup.
