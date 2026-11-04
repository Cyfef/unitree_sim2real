# Isolated deployment-flow simulation

This runner generates a synthetic neutral starting pose, prepares a SuperMimic candidate, and validates the exact C++ controller in a loopback-only MuJoCo simulation. It never opens the physical robot network. It uses the G1 Dex3 model plus the dynamically attached 1.9 kg backpack, nominal and arm-load stress plants, and the selected policy.

Run from `supermimic/` after workspace setup:

```bash
bash simulation/run.sh backpack walk1_subject2 1
bash simulation/run.sh baseline walk1_subject2 1
```

The third argument is policy playback seconds, not total simulation time; FixStand and post-playback observation also run. Results are written below `sessions/offline-*` and excluded from Git. A synthetic reference is suitable only for simulation. It cannot authorize a real-robot launch. For physical deployment, use the fresh-pose onsite `deploy.sh prepare` workflow in the parent [README](../README.md).
