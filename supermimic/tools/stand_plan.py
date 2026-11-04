"""Offline pose-path and arm gravity feedforward planning; never sends DDS."""
import math
import sys
from pathlib import Path
import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]

def alignment_waypoints(model_path, reference_path, ref, nominal, target):
    """Select a bounded arm route; retain the exact requested final pose.

    Evaluate a tucked, forward arm while raising the shoulders, then restore
    pitch/elbows above the goal and approach the final shoulder angles downward.
    Stress margins are hypotheses tested by the verifier, not a calibration.
    """
    sys.path.insert(0, str(ROOT / 'rl_lab_controller/g1_29dof'))
    from sim2sim_check import load_model
    from contract import JOINT_NAMES
    model=load_model(model_path.resolve(),str(reference_path),backpack_mass=1.9)
    data=mujoco.MjData(model)
    data.qpos[3:7]=ref['pelvis_quaternion']
    for side, values in ref['hands'].items():
        for name,q in zip(('thumb_0','thumb_1','thumb_2','index_0','index_1','middle_0','middle_1'),values):
            data.qpos[model.jnt_qposadr[model.joint(f'{side}_hand_{name}_joint').id]]=q
    joints=[model.joint(n).id for n in JOINT_NAMES]
    qa=model.jnt_qposadr[joints]; va=model.jnt_dofadr[joints]
    lo,hi=model.jnt_range[joints].T
    def contacts():
        result=set()
        for c in data.contact:
            if c.dist>=-.003: continue
            bodies=tuple(sorted(int(model.geom_bodyid[g]) for g in c.geom))
            # Fixed finger pose may contain model contact already present at
            # nominal. Reject new penetrations rather than deleting collisions.
            if bodies[0]>0: result.add(bodies)
        return result
    data.qpos[qa]=nominal; mujoco.mj_forward(model,data)
    existing=contacts()
    choices=[]
    for pitch in (-.45,-.60,-.75,-.85):
        for elbow in (1.8,2.0):
            for clearance in (.12,.18):
                tucked=nominal.copy(); tucked[[15,22]]=pitch; tucked[[18,25]]=elbow
                raised=tucked.copy(); raised[[16,23]]=target[[16,23]]+np.array([clearance,-clearance])
                above=target.copy(); above[[16,23]]=raised[[16,23]]
                pts=[nominal,tucked,raised,above,target]
                if any(np.any(q<lo+.03) or np.any(q>hi-.03) for q in pts): continue
                peaks=np.zeros(29); lifting=np.zeros(2); collision=False
                for a,b in zip(pts,pts[1:]):
                    for u in np.linspace(0,1,61):
                        data.qpos[qa]=a+u*(b-a); mujoco.mj_forward(model,data)
                        g=np.abs(data.qfrc_bias[va]); peaks=np.maximum(peaks,g)
                        collision |= bool(contacts()-existing)
                        for k,i in enumerate((16,23)):
                            if (b[i]-a[i])*(1 if i==16 else -1)>1e-6:
                                lifting[k]=max(lifting[k],g[i])
                # Keep the deployed 10.8 Nm roll / 7.2 Nm other arm command
                # budgets. Do not grow a torque limit to make a route pass.
                lift_effort=1.2*lifting+2.
                pitch_effort=1.2*peaks[[15,22]]+.5
                if collision or max(lift_effort)>10.3 or max(pitch_effort)>6.7: continue
                score=max(lift_effort/10.8)+max(pitch_effort/7.2)+.01*sum(np.max(abs(b-a)) for a,b in zip(pts,pts[1:]))
                choices.append((score,pts,dict(pitch_rad=pitch,elbow_rad=elbow,
                    shoulder_clearance_rad=clearance,raising_gravity_peak_Nm=lifting.tolist(),
                    arm_gravity_peak_Nm=peaks[15:].tolist(),
                    stress_arm_mass_scale=1.2,stress_shoulder_friction_Nm=2.,
                    new_body_penetrations=False)))
    if not choices:
        raise ValueError('No tucked-arm alignment route fits existing torque budgets and model collision clearance')
    _,pts,report=min(choices,key=lambda row:row[0])
    report['feasible_routes']=len(choices)
    return pts,report

def plan_stand(model_path, reference_path, ref, waypoints, times, linear_segments=0):
    sys.path.insert(0, str(ROOT / 'rl_lab_controller/g1_29dof'))
    from sim2sim_check import load_model
    from contract import JOINT_NAMES
    model = load_model(model_path.resolve(), str(reference_path), backpack_mass=1.9)
    data = mujoco.MjData(model)
    data.qpos[3:7] = ref['pelvis_quaternion']
    fingers = ('thumb_0','thumb_1','thumb_2','index_0','index_1','middle_0','middle_1')
    for side, values in ref['hands'].items():
        for name, value in zip(fingers, values):
            data.qpos[model.jnt_qposadr[model.joint(f'{side}_hand_{name}_joint').id]] = value
    qa = [model.jnt_qposadr[model.joint(n).id] for n in JOINT_NAMES]
    va = [model.jnt_dofadr[model.joint(n).id] for n in JOINT_NAMES]
    ts, qs, taus = [], [], []
    for k in range(len(times)-1):
        count = math.ceil((times[k+1]-times[k])/.1)
        for i in range(count):
            u = i/count
            t = times[k]+u*(times[k+1]-times[k])
            fraction=u if k<linear_segments else .5-.5*math.cos(math.pi*u)
            q = waypoints[k]+fraction*(waypoints[k+1]-waypoints[k])
            ts.append(t); qs.append(q)
    ts.append(times[-1]); qs.append(waypoints[-1])
    peaks = np.zeros(29)
    for t, q in zip(ts, qs):
        data.qpos[qa] = q
        mujoco.mj_forward(model, data)
        tau = np.zeros(29)
        tau[15:] = data.qfrc_bias[va][15:]
        peaks = np.maximum(peaks, abs(tau))
        tau *= min(1., t/2.)  # load the feedforward smoothly, starting at zero
        taus.append(tau.tolist())
    caps = [30. if i<12 else 15. if i<15 else 3. if i in (20,21,27,28) else 8. for i in range(29)]
    for i in (16,23):
        caps[i] = max(caps[i], float(math.ceil((peaks[i]+2.)/.9)))
        if caps[i] > 12.5:  # at most half of the model's 25 Nm shoulder rating
            raise ValueError(f'shoulder {i}: gravity + margin exceeds conservative stand budget')
    if any(peaks[i] > .9*caps[i] for i in range(15,29)):
        raise ValueError('arm gravity requirement exceeds the stand budget')
    return ts, [q.tolist() for q in qs], taus, caps, peaks.tolist()
