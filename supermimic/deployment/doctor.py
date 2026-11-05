"""Offline runtime check: imports, assets, libraries and empty network namespace."""
import argparse,hashlib,importlib,json,os,subprocess,sys
from pathlib import Path
from common import ROOT,CONTROLLER,MODEL,POLICIES
from isolation import isolated_command

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--network',default=os.environ.get('SUPERMIMIC_NETWORK','enp12s0'))
    p.add_argument('--require-link',action='store_true')
    a=p.parse_args()
    sys.path[:0]=[str(ROOT),str(ROOT/'tools'),str(ROOT.parent/'rl_lab_controller/g1_29dof_deployment')]
    packages={}
    for name in ['numpy','yaml','mujoco','torch','contract','stand_plan','policy_preflight','policy_release','policy_record']:
        m=importlib.import_module(name);packages[name]=getattr(m,'__version__','import_ok')
    assert CONTROLLER.is_file() and os.access(CONTROLLER,os.X_OK),CONTROLLER
    libs=subprocess.run(['ldd',str(CONTROLLER)],capture_output=True,text=True,check=True)
    if 'not found' in libs.stdout:raise RuntimeError(libs.stdout)
    import mujoco
    model=mujoco.MjModel.from_xml_path(str(MODEL))
    motions=sorted(p.stem for p in (ROOT/'assets/motions').glob('*.csv'))
    if len(motions)!=40:raise RuntimeError(f'Expected 40 motions, found {len(motions)}')
    mode, cmd=isolated_command([sys.executable,'-c','import socket,os,json; n=socket.if_nameindex(); assert len(n)==1 and n[0][1]=="lo"; print(json.dumps({"interfaces":n,"uid":os.getuid()}))'],os.environ)
    ns=subprocess.run(cmd,capture_output=True,text=True,timeout=15,check=True)
    net=subprocess.run(['ip','-j','addr','show','dev',a.network],capture_output=True,text=True)
    info=json.loads(net.stdout) if net.returncode==0 else []
    ready=bool(info and 'UP' in info[0]['flags'] and info[0].get('operstate')=='UP' and any(v['family']=='inet' for v in info[0].get('addr_info',[])) and a.network!='lo')
    if a.require_link and not ready:raise RuntimeError(f'{a.network}: physical interface must be UP, have carrier and IPv4 before changing robot control')
    policy_hash = {}
    for name, policy in POLICIES.items():
        for asset in ('policy.onnx', 'policy_meta.json'):
            if not (policy / asset).is_file(): raise FileNotFoundError(policy / asset)
        policy_hash[name] = hashlib.sha256((policy / 'policy.onnx').read_bytes()).hexdigest()
    print(json.dumps({'offline_checks_passed':True,'physical_robot_run':False,'python':sys.executable,'packages':packages,'controller_sha256':hashlib.sha256(CONTROLLER.read_bytes()).hexdigest(),'policy_sha256':policy_hash,'model_nq':model.nq,'motion_count':len(motions),'namespace_mode':mode,'namespace_probe':json.loads(ns.stdout),'network':a.network,'network_ready_for_preflight':ready,'network_info':info,'note':'No DDS initialized; robot connectivity not tested.'},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
