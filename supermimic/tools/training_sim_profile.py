"""Nominal training-model setup for native MuJoCo deployment-flow simulation."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
MODEL=ROOT/'assets/training_model/g1_29dof.xml'


def apply_training_solver(model):
    """Defaults of the training checkout's MJWarpSolverCfg plus env overrides."""
    model.opt.timestep=.005
    model.opt.integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model.opt.cone=mujoco.mjtCone.mjCONE_PYRAMIDAL
    model.opt.solver=mujoco.mjtSolver.mjSOL_NEWTON
    model.opt.iterations=100
    model.opt.ls_iterations=50
    model.opt.tolerance=1e-6
    model.opt.impratio=1.0
    model.opt.ccd_iterations=35
    if hasattr(mujoco.mjtDisableBit, "mjDSBL_MULTICCD"):
        model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_MULTICCD)
    else:
        model.opt.enableflags &= ~int(mujoco.mjtEnableBit.mjENBL_MULTICCD)
    return model

def load_training_model(path):
    root=ET.parse(path).getroot()
    compiler=root.find('compiler')
    compiler.set('meshdir',str((path.parent/compiler.get('meshdir','.')).resolve()))
    ET.SubElement(root.find('worldbody'),'geom',name='floor',type='plane',size='0 0 .05',friction='1 .005 .0001',contype='0',conaffinity='1')
    meta=json.loads((ROOT/'assets/policies/baseline/policy_meta.json').read_text())
    names=meta['joint_names']; arm=np.asarray(meta['kp'])/(10*2*3.1415926535)**2
    joints={j.get('name'):j for j in root.iter('joint')}
    if root.find('actuator') is not None:raise ValueError('expected unactuated training source MJCF')
    actuators=ET.SubElement(root,'actuator')
    for n,a in zip(names,arm):
        j=joints[n];j.set('armature',str(a));j.set('damping','0');j.set('frictionloss','0')
        ET.SubElement(actuators,'general',name=n+'_drive',joint=n,gaintype='fixed',biastype='affine',gainprm='1',biasprm='0 0 0',forcelimited='true',forcerange=j.get('actuatorfrcrange'))
    m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    m.opt.timestep=.005;m.opt.integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST;m.opt.cone=mujoco.mjtCone.mjCONE_PYRAMIDAL
    for g in range(m.ngeom):
        if m.geom_bodyid[g]==0:m.geom_contype[g],m.geom_conaffinity[g]=0,1
        elif m.geom_contype[g] or m.geom_conaffinity[g]:m.geom_contype[g],m.geom_conaffinity[g]=1,0
        m.geom_friction[g,0]=1.
    return apply_training_solver(m)

def grounded_height(model, reference):
    d=mujoco.MjData(model);d.qpos[:3]=0;d.qpos[3:7]=reference['pelvis_quaternion']
    meta=json.loads((ROOT/'assets/policies/baseline/policy_meta.json').read_text())
    for name,q in zip(meta['joint_names'],reference['q']):d.qpos[model.jnt_qposadr[model.joint(name).id]]=q
    mujoco.mj_forward(model,d)
    bottoms=[]
    for side in ('left','right'):
        b=model.body(side+'_ankle_roll_link').id
        for g in range(model.ngeom):
            if model.geom_bodyid[g]!=b or not(model.geom_contype[g] or model.geom_conaffinity[g]):continue
            if model.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH:
                mesh=model.geom_dataid[g];a=model.mesh_vertadr[mesh];n=model.mesh_vertnum[mesh]
                v=model.mesh_vert[a:a+n]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g]
                bottoms.append(float(v[:,2].min()))
            elif model.geom_type[g]==mujoco.mjtGeom.mjGEOM_SPHERE:bottoms.append(float(d.geom_xpos[g,2]-model.geom_size[g,0]))
    if not bottoms:raise ValueError('training model lacks foot collision geometry')
    height=-min(bottoms)
    if not .65<height<.9:raise ValueError(f'unexpected training-model foot height {height}')
    return height


def apply_training_parameters(model):
    """Keep hardware geometry/mass/finger constraints; align 29 body drives."""
    meta=json.loads((ROOT/'assets/policies/baseline/policy_meta.json').read_text())
    for name,kp,scale in zip(meta['joint_names'],meta['kp'],meta['action_scale']):
        j=model.joint(name).id;v=model.jnt_dofadr[j]
        model.dof_armature[v]=kp/(10*2*3.1415926535)**2
        model.dof_damping[v]=0;model.dof_frictionloss[v]=0
        ids=np.flatnonzero(model.actuator_trnid[:,0]==j)
        if len(ids)!=1:raise ValueError('expected one actuator for '+name)
        a=int(ids[0]);model.actuator_gaintype[a]=mujoco.mjtGain.mjGAIN_FIXED
        model.actuator_biastype[a]=mujoco.mjtBias.mjBIAS_AFFINE
        model.actuator_gainprm[a,:]=0;model.actuator_gainprm[a,0]=1
        model.actuator_biasprm[a,:]=0
        # Training: action_scale = 0.25 * effort_limit / kp. The imported
        # hardware XML already caps joint total force; explicitly cap the
        # affine actuator too, matching the training-source simulation.
        effort=4.0*float(kp)*float(scale)
        if not np.isfinite(effort) or effort<=0:
            raise ValueError('invalid training effort for '+name)
        if not model.jnt_actfrclimited[j] or not np.allclose(
                model.jnt_actfrcrange[j], [-effort,effort], rtol=1e-5, atol=1e-5):
            raise ValueError('hardware/training effort mismatch for '+name)
        model.actuator_forcelimited[a]=True
        model.actuator_forcerange[a]=model.jnt_actfrcrange[j]
    model.opt.timestep=.005
    model.opt.integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model.opt.cone=mujoco.mjtCone.mjCONE_PYRAMIDAL
    for g in range(model.ngeom):
        if model.geom_bodyid[g]==0:model.geom_contype[g],model.geom_conaffinity[g]=0,1
        elif model.geom_contype[g] or model.geom_conaffinity[g]:model.geom_contype[g],model.geom_conaffinity[g]=1,0
        model.geom_friction[g,0]=1.
    return apply_training_solver(model)
