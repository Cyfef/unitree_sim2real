"""Batch evaluation of SuperMimic motion clips in the configured simulation."""
import argparse,concurrent.futures,datetime,hashlib,json,logging,os,re,struct,subprocess,sys,threading,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];BUNDLE=ROOT.parent;BASE=Path(__file__).resolve().parent
sys.path.insert(0,str(BUNDLE/'scripts'))
from runtime_env import shared_runtime_env
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--name',required=True);p.add_argument('--workers',type=int,default=8);p.add_argument('--policy',choices=('baseline','backpack'),default='backpack');a=p.parse_args()
if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*',a.name) or not 1<=a.workers<=8:p.error('invalid batch name or concurrency')
B=ROOT/'outputs/simulation'/a.policy/'batches'/a.name;B.mkdir(parents=True,exist_ok=False)
LOG_DIR=B/'logs';LOG_DIR.mkdir()
(LOG_DIR/'launch.json').write_text(json.dumps({
 'pid':os.getpid(),'started':datetime.datetime.now().isoformat(),'batch':str(B),
 'workers':a.workers,'physical_robot_run':False,'model':'hands_backpack',
 'physics_dt':.005,'policy':a.policy},indent=2)+'\n')
logger=logging.getLogger('supermimic_simulation_batch');logger.setLevel(logging.INFO);logger.propagate=False
for handler in [logging.FileHandler(LOG_DIR/'batch.log',mode='x'),logging.StreamHandler(sys.stdout)]:
 handler.setFormatter(logging.Formatter('%(message)s'));logger.addHandler(handler)
clips=sorted(x.stem for x in (ROOT/'assets/motions').glob('*.csv'));assert len(clips)==40
ref=BASE/'poses/standby.json'
paths=[BASE/'run.py',BASE/'bridge.py',ROOT/'tools/prepare_clip.py',ROOT/'tools/stand_plan.py',ROOT/'contract.py',ROOT/'config/recovery_template.json',BASE/'model_tools.py',BASE/'physics.json',BUNDLE/'simulation_core/physics.py',BUNDLE/'scripts/runtime_env.py',ROOT/'tools/model.py',ROOT/'assets/policies'/a.policy/'policy.onnx',ROOT/'assets/policies'/a.policy/'policy_meta.json',BUNDLE/'rl_lab_controller/g1_29dof_deployment/build_simulation/g1_ctrl',BUNDLE/'models/g1_29dof_with_hand_rev_1_0.xml',ref]
paths += sorted((BUNDLE/'models/meshes').glob('*.STL'))
paths += sorted((ROOT/'assets/motions').glob('*.csv'))
sha=lambda x:hashlib.sha256(x.read_bytes()).hexdigest()
hashes={str(x.relative_to(BUNDLE)):sha(x) for x in paths}
manifest=dict(name=a.name,policy=a.policy,clips=clips,workers=a.workers,started=datetime.datetime.now().isoformat(),physical_robot_test=False,model_variant='hands_backpack',imu_convention='deployment',stand_gains='policy',physics_dt=.005,control_dt=.02,start_frame=0,full_clip=True,sha256=hashes)
(B/'manifest.json').write_text(json.dumps(manifest,indent=2));(B/'driver_snapshot.py').write_text(Path(__file__).read_text())
results={c:dict(status='queued') for c in clips};lock=threading.Lock()
env=shared_runtime_env()
def update(c,**kw):
 with lock:
  results[c].update(kw)
  tmp=B/'progress.tmp';tmp.write_text(json.dumps(results,indent=2,ensure_ascii=False));tmp.replace(B/'progress.json')
  logger.info('%s %s %s',datetime.datetime.now().isoformat(timespec='seconds'),c,kw.get('status',''))
def evaluate(c):
 try:
  update(c,status='preparing',started=datetime.datetime.now().isoformat())
  for key,h in hashes.items():
   if sha(BUNDLE/key)!=h:raise RuntimeError('frozen input changed: '+key)
  out=B/'candidates'/c;out.parent.mkdir(exist_ok=True)
  log=LOG_DIR/(c+'_prepare.log')
  with log.open('x') as f:
   q=subprocess.run([sys.executable,str(ROOT/'tools/prepare_clip.py'),'--clip',c,'--policy',a.policy,'--start-seconds','0','--full-clip-simulation','--reference',str(ref),'--output-dir',str(out)],cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=180)
  if q.returncode:
   update(c,status='prepare_failed',reason=log.read_text()[-2000:],finished=datetime.datetime.now().isoformat());return
  prep=json.loads((out/'preparation.json').read_text());cfg=json.loads((out/'candidate.json').read_text())
  with (out/'motion.bin').open('rb') as f:magic,frames,width=struct.unpack('<4sII',f.read(12))
  assert magic==b'SMC1' and width==730 and prep['start_seconds']==0 and abs((frames-1)*.02-prep['max_seconds'])<1e-6
  assert cfg['FSM']['Velocity']['simulation_only'] and cfg['FSM']['Velocity']['max_seconds']==prep['max_seconds']
  run=B/'runs'/c
  update(c,status='running',motion_seconds=prep['max_seconds'],frames=frames,run=str(run.relative_to(ROOT)))
  with (LOG_DIR/(c+'_launch.log')).open('x') as f:
   q=subprocess.run([sys.executable,str(BASE/'run.py'),'--source-session',str(out),'--name',c,'--batch',a.name,'--policy',a.policy,'--stand-gains','policy','--imu-convention','deployment'],cwd=ROOT,env=env,stdout=f,stderr=subprocess.STDOUT)
  summary=run/'simulation/summary.json'
  if not summary.exists():
   update(c,status='infrastructure_error',reason=(LOG_DIR/(c+'_launch.log')).read_text()[-2000:],finished=datetime.datetime.now().isoformat());return
  d=json.loads(summary.read_text());log=(run/'simulation/controller.log').read_text()
  start=re.search(r'\[([^]]+)\].*SuperMimic: MOTION',log);fault=re.search(r'\[([^]]+)\].*Safety stop: (.*)',log)
  secs=(datetime.datetime.fromisoformat(fault[1])-datetime.datetime.fromisoformat(start[1])).total_seconds() if start and fault else None
  passed=d.get('passed') and q.returncode==0 and not d.get('timing_invalid')
  update(c,status='passed' if passed else 'invalid_timing' if d.get('timing_invalid') else 'failed',exit_code=q.returncode,phase='motion_or_post' if start else 'standby' if d.get('policy_entered') else 'preparation',fault=fault[2] if fault else d.get('test_loop_error') or d.get('error'),motion_seconds_to_fault=secs,checks=d.get('supermimic_checks'),timing_invalid=d.get('timing_invalid'),finished=datetime.datetime.now().isoformat())
 except Exception as e:update(c,status='infrastructure_error',reason=repr(e),finished=datetime.datetime.now().isoformat())
with concurrent.futures.ThreadPoolExecutor(max_workers=a.workers) as pool:list(pool.map(evaluate,clips))
counts={s:sum(x['status']==s for x in results.values()) for s in sorted({x['status'] for x in results.values()})}
unchanged={k:sha(BUNDLE/k)==h for k,h in hashes.items()}
final=dict(manifest=manifest,finished=datetime.datetime.now().isoformat(),counts=counts,results=results,frozen_inputs_unchanged=unchanged,complete=len(results)==40 and all(x['status'] not in ['queued','preparing','running'] for x in results.values()))
(B/'results.json').write_text(json.dumps(final,indent=2,ensure_ascii=False))
logger.info('COMPLETE %s',json.dumps(counts))
