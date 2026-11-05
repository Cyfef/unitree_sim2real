"""Prepare a bounded candidate for the shared sim/hardware controller; no DDS."""
import argparse
import json
import os
import subprocess
import sys
from common import ROOT, PROFILE, POLICIES
sys.path.insert(0, str(ROOT.parent / 'scripts'))
from runtime_env import shared_runtime_env

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--clip', required=True)
    p.add_argument('--policy', choices=tuple(POLICIES), required=True)
    p.add_argument('--start-seconds', type=float, default=0.)
    p.add_argument('--max-seconds', type=float, default=5.)
    p.add_argument('--full-clip-simulation', action='store_true')
    p.add_argument('--reference', type=__import__('pathlib').Path, required=True)
    p.add_argument('--output-dir', type=__import__('pathlib').Path, required=True)
    a = p.parse_args()
    out = a.output_dir.resolve()
    cmd = [sys.executable, str(ROOT / 'tools/prepare_clip.py'), '--clip', a.clip,
           '--policy', a.policy, '--start-seconds', str(a.start_seconds), '--max-seconds', str(a.max_seconds),
           '--reference', str(a.reference.resolve()), '--output-dir', str(out)]
    if a.full_clip_simulation: cmd.append('--full-clip-simulation')
    env = shared_runtime_env()
    subprocess.run(cmd, check=True, env=env)
    cfg = json.loads((out / 'candidate.json').read_text())
    cfg['deployment_profile'] = PROFILE
    cfg['supermimic_policy'] = a.policy
    policy = POLICIES[a.policy]
    # FixStand uses the recovery template's validated posture gains. Policy
    # gains belong exclusively to Velocity, not the loaded posture transition.
    cfg['FSM']['Velocity'].update(policy_file=str(policy / 'policy.onnx'),
                                 meta_file=str(policy / 'policy_meta.json'),
                                 motion_file=str(out / 'motion.bin'),
                                 orientation_trace=str(out / 'real_logs/orientation_trace.csv'))
    cfg['Safety'].update(reference=str(out / 'reference.json'), trace_path=str(out / 'real_logs/safety_trace.csv'))
    prep = json.loads((out / 'preparation.json').read_text())
    if json.loads(a.reference.read_text()).get('synthetic_simulation_only'):
        cfg['FSM']['Velocity']['simulation_only'] = True
        prep['simulation_only'] = True
    prep.update(policy=a.policy, deployment_profile=PROFILE, controller='g1_29dof_deployment', status='candidate_not_validated')
    (out / 'candidate.json').write_text(json.dumps(cfg, indent=2) + '\n')
    (out / 'preparation.json').write_text(json.dumps(prep, indent=2) + '\n')
    print(json.dumps({'session': str(out), 'profile': PROFILE, 'simulation_only': prep['simulation_only']}))

if __name__ == '__main__': main()
