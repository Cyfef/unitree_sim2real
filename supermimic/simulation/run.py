"""Loopback-only SuperMimic simulation. Never authorizes hardware use."""
import argparse,json,math,os,subprocess,sys,hashlib,shutil,signal,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];BUNDLE=ROOT.parent;BASE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
sys.path.insert(0,str(ROOT/'tools'))
from model_tools import grounded_height
from model import load_model
sys.path.insert(0,str(ROOT/'deployment'))
sys.path.insert(0,str(BUNDLE/'scripts'))
from isolation import isolated_command
from runtime_env import shared_runtime_env
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--source-session',type=Path,required=True)
p.add_argument('--name',required=True)
p.add_argument('--batch')
p.add_argument('--policy',choices=('baseline','backpack'))
p.add_argument('--stand-gains',choices=['original','policy'],default='policy')
p.add_argument('--seconds',type=float)
p.add_argument('--orientation-fault-at',type=float)
p.add_argument('--imu-convention',choices=['previous_sdk','deployment'],default='deployment')
a=p.parse_args()
if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*',a.name): p.error('invalid run name')
if a.batch and not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*',a.batch): p.error('invalid batch name')
src=a.source_session.resolve()
cfg=json.loads((src/'candidate.json').read_text()); prep=json.loads((src/'preparation.json').read_text())
policy=prep.get('policy')
if policy not in ('baseline','backpack') or (a.policy and a.policy!=policy): p.error('source session policy mismatch')
policy_dir=ROOT/'assets/policies'/policy
controller_root=BUNDLE/'rl_lab_controller/g1_29dof_deployment'
def resolved_asset(value):
 path=Path(value)
 return (path if path.is_absolute() else controller_root/path).resolve()
if resolved_asset(cfg['FSM']['Velocity']['policy_file']) != (policy_dir/'policy.onnx').resolve() or resolved_asset(cfg['FSM']['Velocity']['meta_file']) != (policy_dir/'policy_meta.json').resolve():
 p.error('source session policy assets mismatch')
output_root=ROOT/'outputs/simulation'/policy
out=output_root/'batches'/a.batch/'runs'/a.name if a.batch else output_root/'manual'/a.name
out.mkdir(parents=True,exist_ok=False)
for f in ['motion.bin','reference.json','preparation.json']:shutil.copy2(src/f,out/f)
vel=cfg['FSM']['Velocity'];vel['simulation_only']=True
vel['policy_file']=str(policy_dir/'policy.onnx');vel['meta_file']=str(policy_dir/'policy_meta.json');vel['motion_file']=str(out/'motion.bin')
vel['orientation_trace']=str(out/'orientation_trace.csv')
cfg['Safety']['reference']=str(out/'reference.json');cfg['Safety']['trace_path']=str(out/'safety-trace.csv')
if a.seconds is not None:
 if not 0<a.seconds<=prep['max_seconds']:p.error('invalid bounded seconds')
 vel['max_seconds']=a.seconds
if a.stand_gains=='policy':
 meta=json.loads((policy_dir/'policy_meta.json').read_text())
 cfg['FSM']['FixStand']['kd']=meta['kd']
 # Only preparation gains vary; all software protection thresholds remain unchanged.
(out/'candidate.json').write_text(json.dumps(cfg,indent=2))
model=BUNDLE/'models/g1_29dof_with_hand_rev_1_0.xml'
ref=json.loads((out/'reference.json').read_text())
m=load_model(model,str(out/'reference.json'),False,ref,1.9)
extra=['--hands',str(out/'reference.json'),'--backpack-mass','1.9']
height=grounded_height(m,ref,policy_dir/'policy_meta.json')
controller=BUNDLE/'rl_lab_controller/g1_29dof_deployment/build_simulation/g1_ctrl'
bridge=BASE/'bridge.py'
args=[sys.executable,'-u',str(bridge),'--model',str(model),'--controller',str(controller),'--initial-pose',str(out/'reference.json'),'--config',str(out/'candidate.json'),'--no-stand-button','--stand-time',str(math.ceil(prep['stand_duration_seconds']+4)),'--expect-policy-window-stop','--policy-standby-flow','--output',str(out/'simulation'),'--grounded-start','--root-height',str(height),'--release-duration','10','--zero-time','20','--post-forward-hold','20']
args+=extra+["--imu-convention",a.imu_convention]
if a.orientation_fault_at is not None:
 if not math.isfinite(a.orientation_fault_at) or a.orientation_fault_at<=0:p.error('invalid orientation fault time')
 args+=['--orientation-fault-at',str(a.orientation_fault_at)]
env=shared_runtime_env()
manifest={'protection_profile':'reference_gravity_distance_0p8','physical_robot_test':False,'source_session':str(src),'stand_gains':a.stand_gains,'policy':policy,'model_variant':'hands_backpack','imu_convention':a.imu_convention,'motion_seconds':vel['max_seconds'],'argv':args,'sha256':{str(x.relative_to(BUNDLE)):hashlib.sha256(x.read_bytes()).hexdigest() for x in [bridge,controller,model,BUNDLE/'simulation_core/physics.py',ROOT/'tools/model.py',BASE/'physics.json',BUNDLE/'scripts/runtime_env.py',policy_dir/'policy.onnx',policy_dir/'policy_meta.json',out/'candidate.json',out/'motion.bin']}}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
shutil.copy2(bridge,out/'bridge_snapshot.py')
shutil.copy2(Path(__file__),out/'runner_snapshot.py')
(out/'logs').mkdir()
with (out/'logs/run.log').open('x') as log:
 namespace_mode,command=isolated_command(args,env)
 proc=subprocess.Popen(command,env=env,cwd=BUNDLE/'rl_lab_controller/g1_29dof_deployment',stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 try:
  code=proc.wait(timeout=vel['max_seconds']+180)
 except BaseException:
  try:os.killpg(proc.pid,signal.SIGTERM)
  except ProcessLookupError:pass
  try:proc.wait(timeout=5)
  except subprocess.TimeoutExpired:
   try:os.killpg(proc.pid,signal.SIGKILL)
   except ProcessLookupError:pass
   proc.wait()
  raise
print(json.dumps({'run':str(out),'exit_code':code}),flush=True)
sys.exit(code)
