from pathlib import Path
import hashlib
import json
import sys
import importlib.metadata

A = Path(__file__).resolve().parent
ROOT = A.parent
BUNDLE = ROOT.parent
POLICIES = {name: ROOT / 'assets/policies' / name for name in ('baseline', 'backpack', 'backpack_retrain')}
PROFILE = 'loaded_posture_v5'
CONTROLLER = BUNDLE / 'rl_lab_controller/g1_29dof_deployment/build/g1_ctrl'
BRIDGE = A / 'bridge.py'
MODEL_ROOT = BUNDLE / 'models'
MODEL = MODEL_ROOT / 'g1_29dof_with_hand_rev_1_0.xml'

def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for data in iter(lambda: f.read(1048576), b''):
            h.update(data)
    return h.hexdigest()

def selected_policy(session):
    prep = json.loads((session / 'preparation.json').read_text())
    name = prep.get('policy')
    if name not in POLICIES:
        raise RuntimeError('Unknown candidate policy; prepare a fresh candidate')
    return name, POLICIES[name]

def inputs(session):
    _, policy = selected_policy(session)
    paths = {'controller': CONTROLLER, 'bridge': BRIDGE,
             'policy': policy / 'policy.onnx',
             'metadata': policy / 'policy_meta.json',
             'common': A / 'common.py', 'verifier': A / 'verify.py',
             'prepare_wrapper': A / 'prepare.py', 'launcher': A / 'launch.sh',
             'model_loader': ROOT / 'tools/model.py',
             'preflight': BUNDLE / 'rl_lab_controller/g1_29dof_deployment/policy_preflight.py',
             'recorder': BUNDLE / 'rl_lab_controller/g1_29dof_deployment/policy_record.py',
             'release': BUNDLE / 'rl_lab_controller/g1_29dof_deployment/policy_release.py',
             'joint_state': BUNDLE / 'rl_lab_controller/g1_29dof_deployment/joint_state.py'}
    paths.update({'posture_planner': ROOT / 'tools/stand_plan.py',
                  'clip_preparer': ROOT / 'tools/prepare_clip.py',
                  'recovery_template': ROOT / 'config/recovery_template.json'})
    paths.update({'runtime_env': ROOT / 'config/env.sh',
                  'shared_env': BUNDLE / 'scripts/env.sh',
                  'shared_runtime_env_py': BUNDLE / 'scripts/runtime_env.py', 'isolation': A / 'isolation.py',
                  'doctor': A / 'doctor.py', 'entrypoint': ROOT / 'deploy.sh'})
    paths.update({n: session / n for n in ['candidate.json', 'reference.json', 'preparation.json', 'motion.bin']})
    paths.update({'model/' + str(p.relative_to(MODEL_ROOT)): p for p in MODEL_ROOT.rglob('*') if p.is_file()})
    return paths

def hashes(session):
    return {name: digest(p) for name, p in inputs(session).items()}

def read_candidate(session):
    cfg = json.loads((session / 'candidate.json').read_text())
    prep = json.loads((session / 'preparation.json').read_text())
    policy_name, policy = selected_policy(session)
    if cfg.get('supermimic_policy') != policy_name:
        raise RuntimeError('Candidate/policy mismatch')
    if cfg.get('deployment_profile') != PROFILE or prep.get('deployment_profile') != PROFILE:
        raise RuntimeError('Prepare a fresh loaded_posture_v5 candidate; previous posture validation is obsolete')
    if prep.get('posture_model') != str(MODEL) or prep.get('assumed_backpack_mass_kg') != 1.9:
        raise RuntimeError('Posture planning and validation must use the same hand/backpack model')
    sim = bool(prep.get('simulation_only'))
    if bool(cfg['FSM']['Velocity'].get('simulation_only')) != sim:
        raise RuntimeError('Candidate/preparation execution scope mismatch')
    vel = cfg['FSM']['Velocity']
    # Hash the actual files that the C++ controller will open, not just
    # same-named files next to a copied candidate or validation record.
    bindings = [('policy', vel['policy_file'], policy / 'policy.onnx'),
                ('metadata', vel['meta_file'], policy / 'policy_meta.json'),
                ('motion', vel['motion_file'], session / 'motion.bin'),
                ('reference', cfg['Safety']['reference'], session / 'reference.json'),
                ('orientation trace', vel['orientation_trace'], session / 'real_logs/orientation_trace.csv'),
                ('safety trace', cfg['Safety']['trace_path'], session / 'real_logs/safety_trace.csv')]
    for label, configured, expected in bindings:
        actual = Path(configured)
        if not actual.is_absolute(): actual = CONTROLLER.parent.parent / actual
        if actual.resolve() != expected.resolve():
            raise RuntimeError(f'{label} path does not match this session; prepare a fresh candidate')
    if vel['max_seconds'] != prep['max_seconds']:
        raise RuntimeError('Candidate/preparation duration mismatch')
    return cfg, prep


def runtime_signature():
    return {'python': sys.version, 'executable': str(__import__('pathlib').Path(sys.executable).resolve()),
            'packages': {n: importlib.metadata.version(n) for n in ['numpy','mujoco','torch','PyYAML','cyclonedds']}}
