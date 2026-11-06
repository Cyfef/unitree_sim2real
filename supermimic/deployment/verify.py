"""Validate the exact binary in nominal and loaded/friction stress plants."""
from pathlib import Path
import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
from common import A, ROOT, PROFILE, CONTROLLER, BRIDGE, MODEL, hashes, read_candidate, runtime_signature
from isolation import isolated_command
sys.path.insert(0, str(ROOT.parent / 'scripts'))
from runtime_env import shared_runtime_env

def check(session):
    cfg, prep = read_candidate(session)
    if prep['simulation_only']:
        raise RuntimeError('Full-length simulation candidate cannot authorize hardware')
    record = json.loads((session / 'validation_synced.json').read_text())
    if record.get('status') != 'same_binary_isolated_simulation_passed' or record.get('profile') != PROFILE or record.get('sha256') != hashes(session) or record.get('runtime') != runtime_signature():
        raise RuntimeError('Shared-controller validation missing or assets changed; prepare and verify again')
    if set(record.get('validation_profiles',{}))!={'nominal','arm_stress'} or not all(
            all(p['checks'].values()) and p['sha256']==record['sha256'] for p in record['validation_profiles'].values()):
        raise RuntimeError('Both nominal and arm load/friction stress validations are required')
    if not 5 <= cfg['FSM']['Velocity']['max_seconds'] <= 600:
        raise RuntimeError('Hardware window must be 5–600 seconds')
    return record

def run(session, orientation_fault_at=None, fault=None, fault_at=None, stress=False):
    sys.path.insert(0, str(ROOT / 'tools'))
    from prepare_clip import grounded_height
    cfg, prep = read_candidate(session)
    suffix='_stress' if stress else '_synced'
    output = session / ('simulation'+suffix)
    if output.exists(): raise FileExistsError(output)
    before = hashes(session)
    ref = json.loads((session / 'reference.json').read_text())
    height = grounded_height(MODEL, ref)
    # Only network scope and log destinations differ from the hardware candidate.
    cfg['FSM']['Velocity']['simulation_only'] = True
    cfg['FSM']['Velocity']['orientation_trace'] = str(output / 'orientation_trace.csv')
    cfg['Safety']['trace_path'] = str(output / 'safety_trace.csv')
    simulated = session / ('simulation_config'+suffix+'.json')
    simulated.write_text(json.dumps(cfg, indent=2))
    args = [sys.executable, '-u', str(BRIDGE), '--model', str(MODEL), '--controller', str(CONTROLLER),
            '--initial-pose', str(session / 'reference.json'), '--config', str(simulated),
            '--hands', str(session / 'reference.json'), '--backpack-mass', '1.9',
            '--sim-profile', 'deployment',
            '--no-stand-button', '--stand-time', str(math.ceil(prep['stand_duration_seconds'] +
                prep['stand_progress_allowance_seconds'] + 4)),
            '--expect-policy-window-stop', '--policy-standby-flow', '--output', str(output),
            '--grounded-start', '--root-height', str(height), '--release-duration', '10',
            '--zero-time', '20', '--post-forward-hold', '20', '--imu-convention', 'deployment']
    if orientation_fault_at is not None: args += ['--orientation-fault-at', str(orientation_fault_at)]
    if fault is not None: args += ['--fault', fault, '--fault-at', str(fault_at if fault_at is not None else 6.)]
    if stress: args += ['--arm-mass-scale','1.2','--shoulder-friction','2.0']
    env = shared_runtime_env()
    namespace_mode, command = isolated_command(args, env)
    log_dir = session / 'logs'
    log_dir.mkdir(exist_ok=True)
    with (log_dir / ('verification'+suffix+'.log')).open('x') as log:
        proc = subprocess.Popen(command, cwd=ROOT.parent / 'rl_lab_controller/g1_29dof_deployment',
                                env=env, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        try: code = proc.wait(timeout=prep['stand_duration_seconds'] + prep['max_seconds'] + 180)
        except BaseException:
            os.killpg(proc.pid, signal.SIGTERM)
            try: proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL); proc.wait()
            raise
    summary = json.loads((output / 'summary.json').read_text())
    log = (output / 'controller.log').read_text()
    rows = list(csv.DictReader((output / 'orientation_trace.csv').open())) if (output / 'orientation_trace.csv').exists() else []
    motion = [r for r in rows if r['phase'] == '1']
    unchanged = before == hashes(session)
    audit = {'profile': PROFILE, 'controller': str(CONTROLLER), 'process_returncode': code,
             'physical_robot_run': False, 'namespace_mode': namespace_mode, 'assets_unchanged': unchanged,
             'motion_frames': len(motion), 'expected_frames': round(prep['max_seconds'] / .02),
             'summary': summary, 'fault_injected': orientation_fault_at is not None or fault is not None}
    (session / ('verification_audit_stress.json' if stress else 'verification_audit.json')).write_text(json.dumps(audit, indent=2))
    if audit['fault_injected']:
        raise RuntimeError('Fault-injection diagnostic cannot create hardware validation; inspect verification_audit.json')
    if code != 0 or not summary.get('passed') or summary.get('timing_invalid') or 'Safety stop:' in log or len(motion) != audit['expected_frames'] or not unchanged:
        raise RuntimeError('Shared-controller isolated test did not pass; see verification_audit.json')
    record = {'status': 'same_binary_isolated_simulation_passed', 'profile': PROFILE,
              'simulation_only': prep['simulation_only'], 'checks': summary['supermimic_checks'],
              'conditions': {'model': str(MODEL), 'physics_dt_s': .001, 'state_dt_s': .005, 'policy_dt_s': .02,
                             'self_collision': True, 'dexterous_hands': True, 'backpack': True,
                             'assumed_backpack_mass_kg': 1.9, 'hardware_payload_measured': False,
                             'arm_mass_scale': 1.2 if stress else 1.,
                             'added_shoulder_friction_Nm': 2. if stress else 0.,
                             'actuator_torque_cap_scope': 'MuJoCo model, not a hardware firmware calibration'},
              'sha256': before, 'runtime': runtime_signature(), 'namespace_mode': namespace_mode}
    name = 'validation_arm_stress.json' if stress else 'validation_nominal.json'
    (session / name).write_text(json.dumps(record, indent=2))
    print(json.dumps({'passed': True, 'session': str(session), 'validation': name}))
    return record

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', type=Path, required=True)
    p.add_argument('--check-only', action='store_true')
    p.add_argument('--orientation-fault-at', type=float)
    p.add_argument('--fault', choices=['stale', 'crc', 'mode', 'stand_stall'])
    p.add_argument('--fault-at', type=float)
    a = p.parse_args(); session = a.session.resolve()
    if a.check_only: print(json.dumps(check(session)['checks']))
    elif a.orientation_fault_at is not None or a.fault is not None:
        run(session, a.orientation_fault_at, a.fault, a.fault_at)
    else:
        from concurrent.futures import ThreadPoolExecutor
        with ThreadPoolExecutor(max_workers=2) as executor:
            nominal=executor.submit(run,session)
            stress=executor.submit(run,session,stress=True)
            normal_record,stress_record=nominal.result(),stress.result()
        if normal_record['sha256']!=stress_record['sha256'] or hashes(session)!=normal_record['sha256']:
            raise RuntimeError('Assets changed between paired validations')
        record=dict(normal_record)
        record['validation_profiles']={'nominal':normal_record,'arm_stress':stress_record}
        name='simulation_validation_synced.json' if record['simulation_only'] else 'validation_synced.json'
        (session/name).write_text(json.dumps(record,indent=2)+'\n')
        print(json.dumps({'passed':True,'session':str(session),'validation':name,'profiles':['nominal','arm_stress']}))

if __name__ == '__main__': main()
