"""Root-height calculation for SuperMimic simulation."""
import json
from pathlib import Path
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

def grounded_height(model, reference, metadata_path):
    d=mujoco.MjData(model);d.qpos[:3]=0;d.qpos[3:7]=reference['pelvis_quaternion']
    meta=json.loads(metadata_path.read_text())
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

