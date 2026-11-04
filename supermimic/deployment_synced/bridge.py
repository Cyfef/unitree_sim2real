#!/usr/bin/env python3
"""Exercise the actual C++ controller in a loopback-only MuJoCo simulation.

No hardware access is permitted. Run inside `unshare -Urn`, with lo brought up.
Virtual suspension is removed after policy entry; results use the free robot.
"""
import sys
ROOT=__import__("pathlib").Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT / "tools"))
sys.path.insert(0,str(ROOT / "rl_lab_controller/g1_29dof"))
import argparse
from collections import deque
import csv
import json
import math
from pathlib import Path
import socket
import struct
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
import yaml

import mujoco
import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from joint_state import JOINTS


def load_model(path, hands, suspended=False, initial=None, backpack_mass=None):
    if not hands and backpack_mass is None:
        return mujoco.MjModel.from_xml_path(str(path))
    root = ET.parse(path).getroot()
    compiler = root.find("compiler")
    compiler.set("meshdir", str((path.parent / compiler.get("meshdir", "meshes")).resolve()))
    if backpack_mass is not None:
        torso=next(b for b in root.iter('body') if b.get('name')=='torso_link')
        backpack=ET.SubElement(torso,'body',name='backpack_link',pos='-0.094605 -0.0005 0.165324')
        inertia=np.array([.015510,.010909,.007288,.000017,.000563,.000059])*backpack_mass/1.9
        ET.SubElement(backpack,'inertial',pos='0 0 0',mass=str(backpack_mass),fullinertia=' '.join(map(str,inertia)))
        mesh=(path.parent/ 'meshes/backpack_link.STL').resolve()
        if not mesh.is_file():raise FileNotFoundError(mesh)
        ET.SubElement(root.find('asset'),'mesh',name='diagnostic_backpack_mesh',file=str(mesh))
        ET.SubElement(backpack,'geom',type='mesh',mesh='diagnostic_backpack_mesh',density='0')
    if not any(g.get("name") == "floor" for g in root.iter("geom")):
        ET.SubElement(root.find("worldbody"), "geom", name="floor", type="plane", size="0 0 .05")
    # Finger motors have their own controller on the real robot. This test
    # fixes their measured pose, while retaining their mass and collisions.
    ref = json.loads(Path(hands).read_text())
    if suspended:
        pelvis = next(b for b in root.iter("body") if b.get("name")=="pelvis")
        for joint in list(pelvis.findall("joint")):
            if joint.get("type")=="free": pelvis.remove(joint)
        # Removing the free joint welds pelvis to the world and changes
        # MuJoCo's parent-body collision filtering. Preserve the original
        # floating model's exclusions for the two direct hip-pitch children.
        contact = root.find("contact")
        if contact is None:
            contact = ET.SubElement(root, "contact")
        excluded = {frozenset((e.get("body1"), e.get("body2")))
                    for e in contact.findall("exclude")}
        for side in ("left", "right"):
            hip = side + "_hip_pitch_link"
            if frozenset(("pelvis", hip)) not in excluded:
                ET.SubElement(contact, "exclude", body1="pelvis", body2=hip)
        pelvis.set("pos","0 0 1.1")
        pelvis.set("quat"," ".join(str(q) for q in initial["pelvis_quaternion"]))
    fingers = ("thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1")
    equality = ET.SubElement(root, "equality")
    for side, values in ref["hands"].items():
        for name, q in zip(fingers, values):
            ET.SubElement(equality, "joint", joint1=f"{side}_hand_{name}_joint",
                          polycoef=f"{q} 0 0 0 0", solref=".002 1")
    # Match the joint loss parameters of the official MuJoCo baseline.
    for joint in root.iter("joint"):
        if joint.get("name") in {n + "_joint" for n in JOINTS}:
            joint.set("damping", ".05")
            joint.set("armature", ".01")
            joint.set("frictionloss", ".1" if "wrist" in joint.get("name") else ".2")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    return model


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--imu-convention", choices=("previous_sdk", "deployment"), default="previous_sdk")
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--controller", type=Path, required=True)
    parser.add_argument("--hands", help="pose JSON; use with the official Dex3 model XML")
    parser.add_argument("--backpack-mass", type=float, help="assumed AGX backpack mass kg; vendor nominal 1.9, not measured")
    parser.add_argument("--speed", type=float, default=.3, help="forward command in m/s")
    parser.add_argument("--config", type=Path, help="controller configuration (JSON for suspended tests)")
    parser.add_argument("--initial-pose", type=Path, help="initial body pose JSON")
    parser.add_argument("--grounded-start", action="store_true", help="start measured pose with feet on floor, rather than lowering from suspension")
    parser.add_argument("--root-height", type=float, help="grounded root height inferred from foot geometry, meters")
    parser.add_argument("--stand-time", type=float, default=3.)
    parser.add_argument("--suspended", action="store_true", help="fix pelvis; test the stand transition only")
    parser.add_argument("--expect-policy-window-stop", action="store_true",
                        help="SuperMimic: enter policy and verify its bounded-window disabled shutdown")
    parser.add_argument("--policy-standby-flow", action="store_true", help="R1+A standby, R1+X motion, verify three seconds of final hold")
    parser.add_argument("--no-stand-button", action="store_true", help="test configured one-shot automatic stand without simulated L2+Up")
    parser.add_argument("--fault", choices=("stale","crc","torque","mode","stand_stall"))
    parser.add_argument("--fault-at", type=float, default=6., help="simulation time of fault injection")
    parser.add_argument("--mass-scale", type=float, default=1.)
    parser.add_argument("--arm-mass-scale", type=float, default=1., help="isolated stress: scale arm/hand mass and inertia")
    parser.add_argument("--shoulder-friction", type=float, default=0., help="isolated stress: additional shoulder-roll friction, Nm")
    parser.add_argument("--early-policy", action="store_true", help="also press policy before the stand trajectory finishes")
    parser.add_argument("--support-delay", type=float, default=0., help="hold suspension after policy entry, seconds")
    parser.add_argument("--policy-delay", type=float, default=0., help="extra fixed-stand time after feet contact")
    parser.add_argument("--preload-support", type=float, default=1.,
                        help="fraction of virtual support retained before policy entry; fades over 5 seconds after feet contact")
    parser.add_argument("--release-duration", type=float, default=4., help="seconds to fade suspension force")
    parser.add_argument("--retained-support", type=float, default=0., help="synthetic support fraction retained after unloading; not measured hoist tension")
    parser.add_argument("--forward-duration", type=float, default=10., help="seconds of forward command, followed by zero")
    parser.add_argument("--post-forward-hold", type=float, default=6., help="zero-command observation after the forward pulse, seconds")
    parser.add_argument("--backward-pull-force", type=float, default=0., help="synthetic backward force at torso COM in N, along initial robot heading")
    parser.add_argument("--pull-after-forward-start", type=float, default=1., help="pull onset relative to forward command, seconds")
    parser.add_argument("--pull-duration", type=float, default=.25)
    parser.add_argument("--zero-time", type=float, default=.5, help="zero-speed hold after suspension release")
    parser.add_argument("--zero-only", action="store_true", help="test balance without any forward command")
    parser.add_argument("--ankle-breakaway", type=float, default=0.,
                        help="synthetic left-ankle starting friction in Nm, released after initial slip")
    parser.add_argument("--ankle-friction", type=float, default=0.,
                        help="persistent left ankle pitch friction stress in Nm; not a measured calibration")
    parser.add_argument("--physics-substeps", type=int, choices=(1,2,4), default=1,
                        help="motor PD integration steps per 2ms DDS bridge update")
    parser.add_argument("--command-delay", type=float, default=0., help="synthetic motor command delay, seconds")
    parser.add_argument("--wrist-torque-scale", type=float, default=1.,
                        help="synthetic wrist torque headroom to test measured overloads")
    parser.add_argument("--gdb", action="store_true", help="capture a controller crash backtrace")
    parser.add_argument("--output", type=Path, required=True, help="new directory for logs")
    parser.add_argument("--sim-profile", choices=("baseline", "deployment"), default="baseline")
    parser.add_argument("--orientation-fault-at", type=float, help="isolated test: publish inverted IMU after this simulation time")
    args = parser.parse_args()
    if not (1<=args.arm_mass_scale<=1.4 and 0<=args.shoulder_friction<=4):
        parser.error('arm stress outside diagnostic range')
    if (args.arm_mass_scale!=1 or args.shoulder_friction) and args.sim_profile!='deployment':
        parser.error('arm stress requires the loaded deployment model')
    if not 0 < args.speed <= .5:
        parser.error("test speed must be in (0, 0.5] m/s")
    if args.backpack_mass is not None and (not args.hands or not .5<=args.backpack_mass<=4.):
        parser.error('backpack test requires hands and assumed mass in [0.5,4] kg')
    if not (0<=args.support_delay<=300 and .5<=args.release_duration<=60 and .5<=args.zero_time<=300):
        parser.error("invalid suspension/zero-speed timing")
    if not (0<=args.policy_delay<=300 and 0<=args.preload_support<=1):
        parser.error("invalid pre-policy support timing/fraction")
    if not (0<=args.retained_support<=1 and .5<=args.forward_duration<=30 and 6<=args.post_forward_hold<=120 and
            0<=args.backward_pull_force<=200 and 0<=args.pull_after_forward_start<=30 and .05<=args.pull_duration<=2):
        parser.error("invalid synthetic support/pull or forward duration")
    if args.backward_pull_force and (args.suspended or args.zero_only):
        parser.error("backward pull test requires walking, free pelvis")
    if args.grounded_start and (not args.initial_pose or args.suspended or args.root_height is None or not .65<args.root_height<.9):
        parser.error("grounded start requires measured pose, free pelvis and root height in (0.65, 0.9)")
    if args.root_height is not None and not args.grounded_start:
        parser.error("root height requires grounded start")
    if args.preload_support<1 and ((args.policy_delay<5 and not args.grounded_start) or not args.initial_pose or args.suspended):
        parser.error("preloading needs an initial pose, free pelvis, and at least 5 seconds policy delay")
    if not 0<=args.ankle_breakaway<=10:
        parser.error("ankle breakaway friction must be in [0, 10] Nm")
    if not 0<=args.ankle_friction<=10 or (args.ankle_friction and args.ankle_breakaway):
        parser.error("persistent ankle friction must be in [0, 10] Nm and cannot combine with breakaway")
    if not (0<=args.command_delay<=.02 and 1<=args.wrist_torque_scale<=1.4):
        parser.error("invalid delay or wrist torque stress setting")
    if not 0<args.fault_at<=300:
        parser.error("fault time must be in (0, 300] seconds")
    if args.suspended and not (args.hands and args.initial_pose and args.config):
        parser.error("suspended tests need --hands, --initial-pose and --config")
    if args.policy_standby_flow and (args.suspended or not args.expect_policy_window_stop):
        parser.error("standby flow requires grounded window validation")
    if args.expect_policy_window_stop and not args.config:
        parser.error("SuperMimic window check requires --config")
    if {name for _, name in socket.if_nameindex()} != {"lo"}:
        parser.error("refusing to run outside a loopback-only network namespace")
    args.output.mkdir(parents=True, exist_ok=False)
    initial = json.loads(args.initial_pose.read_text()) if args.initial_pose else None
    if args.sim_profile == 'deployment' and (not args.hands or args.backpack_mass is None):
        parser.error('deployment validation requires measured fingers and an explicit backpack mass')
    model = load_model(args.model.resolve(), args.hands, False, initial, args.backpack_mass)
    if args.mass_scale != 1 or args.command_delay or args.wrist_torque_scale != 1 or args.ankle_breakaway or args.ankle_friction or args.backward_pull_force or args.suspended:
        parser.error("baseline forbids stress/model overrides")
    baseline=mujoco.MjModel.from_xml_path(str(ROOT/'simulation_old_model_v1/model/g1/scene_29dof.xml'))
    if args.hands and args.sim_profile == 'baseline':
        # Model-only comparison: keep body drive losses and solver options frozen.
        for field in dir(baseline.opt):
            if field.startswith('_'):continue
            value=getattr(baseline.opt,field)
            if isinstance(value,np.ndarray):getattr(model.opt,field)[:]=value
            elif isinstance(value,(int,float)):setattr(model.opt,field,value)
        for name in [n+'_joint' for n in JOINTS]:
            v=model.jnt_dofadr[model.joint(name).id];ov=baseline.jnt_dofadr[baseline.joint(name).id]
            for field in ['dof_armature','dof_damping','dof_frictionloss']:
                getattr(model,field)[v]=getattr(baseline,field)[ov]
    bridge_dt=.005
    args.physics_substeps=5 if args.sim_profile == 'deployment' else 1
    model.opt.timestep=bridge_dt/args.physics_substeps
    model.opt.integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST if args.sim_profile == 'deployment' else mujoco.mjtIntegrator.mjINT_EULER
    if args.sim_profile == 'baseline':
        for g in range(model.ngeom):
            if model.geom_bodyid[g]==0:model.geom_contype[g],model.geom_conaffinity[g]=0,1
            elif model.geom_contype[g] or model.geom_conaffinity[g]:model.geom_contype[g],model.geom_conaffinity[g]=1,0
    for b in range(model.nbody):
        name=model.body(b).name
        if any(name.startswith(side+'_'+part) for side in ('left','right') for part in ('shoulder','elbow','wrist','hand')):
            model.body_mass[b]*=args.arm_mass_scale
            model.body_inertia[b]*=args.arm_mass_scale
    for side in ('left','right'):
        model.dof_frictionloss[model.jnt_dofadr[model.joint(side+'_shoulder_roll_joint').id]]+=args.shoulder_friction
    if args.arm_mass_scale!=1:
        mujoco.mj_setConst(model,mujoco.MjData(model))
    data = mujoco.MjData(model)
    frozen_shoulders=None
    names = [name + "_joint" for name in JOINTS]
    qadr = [model.jnt_qposadr[model.joint(n).id] for n in names]
    vadr = [model.jnt_dofadr[model.joint(n).id] for n in names]
    original_ankle_friction=float(model.dof_frictionloss[vadr[4]])
    original_ankle_impedance=model.dof_solimp[vadr[4]].copy()
    ankle_slipped=False
    if args.ankle_breakaway:
        model.dof_frictionloss[vadr[4]]=args.ankle_breakaway
        # MuJoCo's default soft friction permits creep instead of sticking.
        model.dof_solimp[vadr[4],:2]=.9999
    if args.ankle_friction:
        model.dof_frictionloss[vadr[4]]=args.ankle_friction
        model.dof_solimp[vadr[4],:2]=.9999
    actuators = []
    for name in names:
        matches = np.flatnonzero(model.actuator_trnid[:, 0] == model.joint(name).id)
        if len(matches) != 1:
            raise RuntimeError(f"expected one actuator for {name}, got {len(matches)}")
        actuators.append(int(matches[0]))
    meta=json.loads((ROOT/'assets/policy/policy_meta.json').read_text())
    effort=4*np.asarray(meta['kp'])*np.asarray(meta['action_scale'])
    # An inactive MuJoCo ctrlrange is commonly [0,0], NOT a zero torque budget.
    limited=model.actuator_ctrllimited[actuators].astype(bool)
    torque_lo=np.maximum(-effort,np.where(limited,model.actuator_ctrlrange[actuators,0],-np.inf))
    torque_hi=np.minimum(effort,np.where(limited,model.actuator_ctrlrange[actuators,1],np.inf))
    if np.any(torque_lo>=torque_hi):raise ValueError("invalid enabled actuator torque bounds")
    (args.output/'plant.json').write_text(json.dumps(dict(
        integrator=int(model.opt.integrator),timestep=float(model.opt.timestep),
        tolerance=float(model.opt.tolerance),mass=float(model.body_mass.sum()),
        joint_names=names,armature=model.dof_armature[vadr].tolist(),
        damping=model.dof_damping[vadr].tolist(),frictionloss=model.dof_frictionloss[vadr].tolist(),
        torque_lo=torque_lo.tolist(),torque_hi=torque_hi.tolist(),
        mechanical_parameters_fixed=True,explicit_pd=True,
        model_path=str(args.model.resolve()),dexterous_hands=bool(args.hands),
        self_collisions=args.sim_profile=='deployment',backpack_mass_kg=args.backpack_mass,
        arm_mass_scale=args.arm_mass_scale,added_shoulder_friction_Nm=args.shoulder_friction),indent=2))
    for i in (20,21,27,28):
        model.actuator_forcerange[actuators[i]]*=args.wrist_torque_scale
        model.jnt_actfrcrange[model.joint(names[i]).id]*=args.wrist_torque_scale
    if initial:
        for adr,q in zip(qadr,initial["q"]): data.qpos[adr]=q
        if not args.suspended:
            data.qpos[3:7]=initial["pelvis_quaternion"]
            data.qpos[2]=args.root_height if args.grounded_start else 1.1
    if args.hands and args.sim_profile != "training":
        ref = json.loads(Path(args.hands).read_text())
        fingers = ("thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1")
        for side, values in ref["hands"].items():
            for name, q in zip(fingers, values):
                data.qpos[model.jnt_qposadr[model.joint(f"{side}_hand_{name}_joint").id]] = q
    mujoco.mj_forward(model, data)
    quat=data.xquat[model.body("pelvis").id]
    yaw=math.atan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),1-2*(quat[2]**2+quat[3]**2))
    backward_direction=np.array([-math.cos(yaw),-math.sin(yaw),0.])
    ChannelFactoryInitialize(0, "lo")
    crc = CRC()
    lock = threading.Lock()
    latest = [None, 0.]
    command_history=deque(maxlen=1000)
    invalid = []

    def on_command(command):
        if crc.Crc(command) != command.crc:
            invalid.append("invalid command CRC")
            return
        with lock:
            latest[:] = [command, time.monotonic()]
            command_history.append((latest[1],command))

    subscriber = ChannelSubscriber("rt/lowcmd", LowCmd_)
    subscriber.Init(on_command, 1)
    publisher = ChannelPublisher("rt/lowstate", LowState_)
    publisher.Init()
    state = unitree_hg_msg_dds__LowState_()
    state.mode_machine, state.mode_pr = 5, 0
    rows = []
    joint_rows=[]
    max_tau = np.zeros(29)
    max_speed = np.zeros(29)
    mass = float(sum(model.body_mass))
    pelvis = model.body("pelvis").id
    suspension_body = model.body("torso_link").id
    reason = None
    stand_started=False
    max_wall_lag=0.
    excessive_lag_since=None
    timing_invalid=False
    with (args.output / "controller.log").open("x") as log:
        controller_args=[str(args.controller.resolve()), "--network", "lo"]
        if args.config:
            # Network isolation does not isolate files. Keep simulated safety
            # traces from overwriting a physical session's configured output.
            simulation_config=yaml.safe_load(args.config.read_text())
            if simulation_config.get('Safety') is not None:
                simulation_config['Safety']['trace_path']=str((args.output/'safety-trace.csv').resolve())
            config_path=args.output/'simulation-config.json'
            config_path.write_text(json.dumps(simulation_config,indent=2)+'\n')
            controller_args += ["--config",str(config_path.resolve())]
        if args.gdb:
            controller_args=["gdb","-q","--batch","-ex","set pagination off","-ex","run",
                             "-ex","thread apply all bt","--args"]+controller_args
        process = subprocess.Popen(controller_args,
                                   stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT)
        start = time.monotonic()
        try:
            feet_contact_at=args.stand_time+.5 if args.grounded_start else args.stand_time+4.5
            policy_at=feet_contact_at+args.policy_delay
            release_at=policy_at+args.support_delay+args.release_duration
            forward_at=release_at+args.zero_time
            if args.expect_policy_window_stop:
                window=yaml.safe_load(args.config.read_text())["FSM"]["Velocity"]["max_seconds"]
                duration=policy_at+window+(9.5 if args.policy_standby_flow else 2.0)
            else:
                duration=args.stand_time+7.5 if args.suspended else forward_at+args.forward_duration+args.post_forward_hold
            for step in range(int(duration/bridge_dt)):
                t = step * bridge_dt
                with lock:
                    command, received = latest
                    if args.command_delay:
                        cutoff=time.monotonic()-args.command_delay
                        while len(command_history)>1 and command_history[1][0]<=cutoff:
                            command_history.popleft()
                        command,received=(command_history[0][1],command_history[0][0]) if (
                            command_history and command_history[0][0]<=cutoff) else (None,0.)
                fresh = command is not None and time.monotonic() - received < .05
                data.ctrl[:] = 0
                for i, (qa, va, actuator) in enumerate(zip(qadr, vadr, actuators)):
                    motor = command.motor_cmd[i] if fresh else None
                    if motor is not None and motor.mode:
                        if args.sim_profile.startswith("training"):
                            # Native affine actuator: PD velocity derivative participates in implicitfast.
                            model.actuator_biasprm[actuator,1:3]=[-motor.kp,-motor.kd]
                            data.ctrl[actuator]=motor.tau+motor.kp*motor.q+motor.kd*motor.dq
                        else:
                            data.ctrl[actuator] = motor.tau + motor.kp * (motor.q-data.qpos[qa]) + motor.kd * (motor.dq-data.qvel[va])
                    elif args.sim_profile.startswith("training"):
                        model.actuator_biasprm[actuator,1:3]=0
                # Virtual suspension is used only for the documented startup.
                support = 0. if args.suspended else min(1., max(0., (release_at-t)/args.release_duration))
                preload_progress=1. if args.grounded_start else min(1.,max(0.,(t-feet_contact_at)/5.))
                support*=1-(1-args.preload_support)*preload_progress
                if not args.suspended: support=max(support,args.retained_support)
                data.xfrc_applied[:] = 0
                if not args.suspended:
                    height=.793
                    if args.grounded_start: height=args.root_height
                    elif initial: height=1.1-.307*min(1.,max(0.,(t-args.stand_time-2.5)/2.))
                    data.xfrc_applied[suspension_body, :3] = support * np.array([
                        -20*data.qpos[0]-10*data.qvel[0],
                        -20*data.qpos[1]-10*data.qvel[1],
                        mass*9.81 + 200*(height-data.qpos[2])-100*data.qvel[2]])
                    if forward_at+args.pull_after_forward_start<=t<forward_at+args.pull_after_forward_start+args.pull_duration:
                        data.xfrc_applied[suspension_body,:3]+=args.backward_pull_force*backward_direction
                if command is not None and any(m.kp>0 for m in command.motor_cmd[:29]): stand_started=True
                # The suspended test starts at the measured stationary pose
                # when position control is engaged. Its setup fixture is
                # released for all 29 body joints at that point.
                if command is not None and (not initial or stand_started):
                    for substep in range(args.physics_substeps):
                        # The embedded motor servo evaluates PD between DDS
                        # packets; hold targets, not the computed torque.
                        if substep:
                            for i,(qa,va,actuator) in enumerate(zip(qadr,vadr,actuators)):
                                motor=command.motor_cmd[i] if fresh else None
                                data.ctrl[actuator]=(motor.tau+motor.kp*(motor.q-data.qpos[qa])+
                                    motor.kd*(motor.dq-data.qvel[va])) if motor is not None and motor.mode else 0.
                        data.ctrl[actuators]=np.clip(data.ctrl[actuators],torque_lo,torque_hi)
                        mujoco.mj_step(model,data)
                        max_tau=np.maximum(max_tau,np.abs(data.qfrc_actuator[vadr]))
                        max_speed=np.maximum(max_speed,np.abs(data.qvel[vadr]))
                        # Simplified stick/slip stress case, not a diagnosis.
                        if (args.ankle_breakaway and not ankle_slipped and
                                data.ctrl[actuators[4]]>.9*args.ankle_breakaway and data.qvel[vadr[4]]>.03):
                            ankle_slipped=True
                            model.dof_frictionloss[vadr[4]]=original_ankle_friction
                            model.dof_solimp[vadr[4]]=original_ankle_impedance
                if args.fault=='stand_stall' and t>=args.fault_at:
                    if frozen_shoulders is None:
                        frozen_shoulders=[float(data.qpos[qadr[i]]) for i in (16,23)]
                    for i,q in zip((16,23),frozen_shoulders):
                        data.qpos[qadr[i]]=q
                        data.qvel[vadr[i]]=0
                    mujoco.mj_forward(model,data)
                for i, (qa, va) in enumerate(zip(qadr, vadr)):
                    motor = state.motor_state[i]
                    motor.mode = int(fresh and command.motor_cmd[i].mode != 0)
                    motor.q, motor.dq = float(data.qpos[qa]), float(data.qvel[va])
                    motor.tau_est = float(data.qfrc_actuator[va])
                    max_tau[i] = max(max_tau[i], abs(motor.tau_est))
                    max_speed[i] = max(max_speed[i], abs(motor.dq))
                state.imu_state.quaternion = list(data.xquat[pelvis])
                velocity = np.zeros(6)
                mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, pelvis, velocity, 1)
                state.imu_state.gyroscope = list(data.qvel[3:6] if args.imu_convention=="previous_sdk" else velocity[:3])
                # Real wireless layout: little-endian buttons at bytes 2..3,
                # forward axis at bytes 20..23. Generate rising button edges.
                keys = 0
                if not args.no_stand_button and 2<t<3:
                    keys = (1<<5) | ((1<<12) if t>2.5 else 0)
                elif args.early_policy and 7<t<8:
                    keys = (1<<0) | ((1<<10) if t>7.5 else 0)
                elif args.policy_standby_flow and policy_at-.5<t<policy_at+.5:
                    keys = (1<<0) | ((1<<8) if t>policy_at else 0)  # R1+A: static policy standby
                elif args.policy_standby_flow and policy_at+5.5<t<policy_at+6.5:
                    keys = (1<<0) | ((1<<10) if t>policy_at+6 else 0)  # R1+X: play
                elif not args.policy_standby_flow and (not args.suspended or args.expect_policy_window_stop) and policy_at-.5<t<policy_at+.5:
                    keys = (1<<0) | ((1<<10) if t>policy_at else 0)
                state.wireless_remote[0:2] = [0xfe, 0xef]
                state.wireless_remote[2:4] = list(struct.pack("<H", keys))
                forward_input=args.speed if not args.zero_only and forward_at<t<forward_at+args.forward_duration else 0.
                state.wireless_remote[20:24] = list(struct.pack("<f", forward_input))
                state.tick = step
                if args.orientation_fault_at is not None and t >= args.orientation_fault_at:
                    state.imu_state.quaternion = [0.,1.,0.,0.]  # valid but deliberately inverted test IMU
                if args.fault and t>args.fault_at:
                    if args.fault=="torque": state.motor_state[3].tau_est=200.
                    if args.fault=="mode": state.mode_pr=1
                state.crc = crc.Crc(state)
                if args.fault=="crc" and t>args.fault_at: state.crc ^= 1
                if not (args.fault=="stale" and t>args.fault_at): publisher.Write(state)
                if step % 50 == 0:
                    quat = data.xquat[pelvis]
                    tilt = math.acos(float(np.clip(1-2*(quat[1]**2+quat[2]**2), -1, 1)))
                    world_velocity=np.zeros(6)
                    mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_BODY,pelvis,world_velocity,0)
                    rows.append([t, *data.xpos[pelvis], *world_velocity[3:], tilt, support,
                                 struct.unpack("<f",bytes(state.wireless_remote[20:24]))[0],
                                 time.monotonic()-received if command is not None else math.nan])
                    if command is not None:
                        joint_rows.append([t]+[float(data.qpos[a]) for a in qadr]+
                            [m.q for m in command.motor_cmd[:29]]+[m.kp for m in command.motor_cmd[:29]]+
                            [m.mode for m in command.motor_cmd[:29]])
                # A latched fault disables motors before writing its trace.
                # End the plant test on that observed output, rather than
                # letting an unsupported disabled robot fall during file I/O.
                if args.fault and (process.poll() is not None or (
                        t>args.fault_at and command is not None and
                        all(m.mode==0 for m in command.motor_cmd))): break
                if args.expect_policy_window_stop and t>policy_at and process.poll() is not None:
                    break
                if invalid or process.poll() is not None:
                    raise RuntimeError(f"controller exited: {process.poll()}, {invalid}")
                if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
                    raise RuntimeError(f"nonfinite simulated state at {t:.3f}s")
                if step % 10 == 0:
                    current_log=(args.output/"controller.log").read_text()
                    if "Reference orientation protection:" in current_log and "Change state from Velocity to Passive" in current_log:
                        raise RuntimeError("reference gravity distance >0.8: FSM switched to Passive")
                delay = start + (step+1)*bridge_dt - time.monotonic()
                max_wall_lag=max(max_wall_lag,-delay)
                if t>policy_at:
                    if -delay>.02:
                        if excessive_lag_since is None: excessive_lag_since=time.monotonic()
                        if time.monotonic()-excessive_lag_since>.1:
                            timing_invalid=True
                            raise RuntimeError("INVALID TIMING: simulation lag >20 ms persisted >100 ms")
                    else: excessive_lag_since=None
                if delay>0:
                    time.sleep(delay)
        except Exception as exc:
            reason = str(exc)
        finally:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            publisher.Close()
            subscriber.Close()
    with (args.output/"state.csv").open("x", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "tilt_rad", "support_fraction",
                         "forward_axis","command_age_s"])
        writer.writerows(rows)
    with (args.output/"joints.csv").open("x", newline="") as stream:
        writer=csv.writer(stream)
        writer.writerow(["t"]+[f"q_{i}" for i in range(29)]+[f"desired_{i}" for i in range(29)]+
                        [f"kp_{i}" for i in range(29)]+[f"mode_{i}" for i in range(29)])
        writer.writerows(joint_rows)
    before = next((r for r in rows if r[0]>=forward_at), None)
    after = next((r for r in rows if r[0]>=forward_at+args.forward_duration), None)
    delta = after[1]-before[1] if before and after else None
    zero_rows=[r for r in rows if r[0]>=release_at and (args.zero_only or r[0]<forward_at)]
    zero_drift=max((math.hypot(r[1]-zero_rows[0][1],r[2]-zero_rows[0][2]) for r in zero_rows),default=0.)
    post_rows=[r for r in rows if r[0]>=forward_at+args.forward_duration+2]
    post_drift=max((math.hypot(r[1]-post_rows[0][1],r[2]-post_rows[0][2]) for r in post_rows),default=None)
    if args.zero_only and not reason and zero_drift>.15:
        reason=f"zero-command horizontal drift {zero_drift:.3f}m exceeds 0.15m"
    exit_code=process.returncode
    controller_log=(args.output/"controller.log").read_text()
    stand_reached="FixStand reached:" in controller_log
    policy_entered="to Velocity" in controller_log
    loop_error=reason
    completed_schedule=step >= int(duration/bridge_dt)-1
    expected_policy=not args.suspended and (not args.fault or args.fault_at>policy_at)
    if not reason and expected_policy and not policy_entered:
        reason="controller never entered Velocity; this is not a policy validation"
    if not reason and args.suspended and not args.fault and not stand_reached:
        reason="controller never reached the stand readiness gate"
    if args.gdb:
        debug_log=controller_log
        if "SIGSEGV" in debug_log: exit_code=-11
        elif "exited with code 01" in debug_log: exit_code=1
    summary = {"orientation_fault_at":args.orientation_fault_at,"protection_profile":"reference_gravity_distance_0p8","imu_convention":args.imu_convention,"sim_profile":args.sim_profile,"physics_dt_s":model.opt.timestep,"state_publish_dt_s":bridge_dt,"timing_invalid":timing_invalid,"self_collisions_enabled":args.sim_profile == "deployment","error":reason, "mass_kg":mass, "command_speed_m_s":args.speed, "forward_distance_m":delta,
               "final_xyz":list(data.xpos[pelvis]), "peak_estimated_torque_Nm":max_tau.tolist(),
               "peak_joint_speed_rad_s":max_speed.tolist(),
               "physical_robot_test":False, "finger_pose_fixed":bool(args.hands) and args.sim_profile != "training","pelvis_fixed":args.suspended,
               "assumed_backpack_mass_kg":args.backpack_mass,"hardware_backpack_mass_measured":False,
               "stand_reached":stand_reached,"policy_entered":policy_entered,
               "setup_pose_held_until_stand":bool(initial),"mass_scale":args.mass_scale,
               "arm_mass_scale":args.arm_mass_scale,"added_shoulder_friction_Nm":args.shoulder_friction,
               "grounded_start":args.grounded_start,"initial_root_height_m":args.root_height,
               "zero_only":args.zero_only,"zero_time_s":args.zero_time,
               "support_delay_s":args.support_delay,"release_duration_s":args.release_duration,
               "policy_delay_s":args.policy_delay,"preload_support_fraction":args.preload_support,
               "retained_support_fraction":args.retained_support,"forward_duration_s":args.forward_duration,
               "post_forward_hold_s":args.post_forward_hold,"post_forward_horizontal_drift_m":post_drift,
               "post_forward_peak_tilt_rad":max((r[7] for r in post_rows),default=None),
               "synthetic_backward_pull_N":args.backward_pull_force,"pull_duration_s":args.pull_duration,
               "pull_after_forward_start_s":args.pull_after_forward_start,
               "synthetic_ankle_breakaway_Nm":args.ankle_breakaway,"ankle_slipped":ankle_slipped,
               "synthetic_persistent_ankle_friction_Nm":args.ankle_friction,
               "physics_substeps":args.physics_substeps,
               "command_delay_s":args.command_delay,"wrist_torque_scale":args.wrist_torque_scale,
               "injected_fault":args.fault,"fault_at_s":args.fault_at if args.fault else None,
               "max_wall_lag_s":max_wall_lag,"max_zero_command_drift_m":zero_drift,
               "controller_exit":exit_code,"final_command_modes":[m.mode for m in latest[0].motor_cmd] if latest[0] else None}
    if args.suspended:
        target=json.loads(args.config.read_text())["FSM"]["FixStand"]["qs"][-1]
        summary["final_joint_error_rad"]=max(abs(data.qpos[adr]-q) for adr,q in zip(qadr,target))
    if not reason and not args.fault:
        if args.suspended and summary["final_joint_error_rad"]>.15:
            reason="final stand joint error exceeds 0.15 rad"
        elif not args.suspended and not args.zero_only and (delta is None or delta<.5):
            reason=f"forward displacement {delta} m did not reach 0.5 m"
    if not reason and exit_code!=(1 if args.fault else 0):
        reason=f"unexpected controller exit code {exit_code}"
    if not reason and any(summary["final_command_modes"] or [1]):
        reason="final motor command modes were not all disabled"
    summary["error"]=reason
    summary["passed"]=reason is None
    if args.expect_policy_window_stop:
        expected="SuperMimic: configured motion window finished"
        checks={"stand_reached":stand_reached,"policy_entered":policy_entered,
                "window_stop_logged":expected in controller_log,
                "controller_exit_0":exit_code==0,
                "all_motors_disabled":not any(summary["final_command_modes"] or [1]),
                "no_bridge_error":not invalid}
        if yaml.safe_load(args.config.read_text())["FSM"]["FixStand"].get("nominal_time") is not None:
            checks["nominal_stand_stable"]="FixStand: ALIGN_FIRST_FRAME; nominal stand stable" in controller_log
        if args.policy_standby_flow:
            checks.pop("window_stop_logged")
            checks.update(standby_pre_entered="SuperMimic: STANDBY_PRE;" in controller_log,
                          standby_ready="STANDBY_PRE ready;" in controller_log,
                          motion_started="SuperMimic: MOTION" in controller_log,
                          final_hold_entered="SuperMimic: STANDBY_POST;" in controller_log,
                          full_test_schedule_completed=completed_schedule,
                          no_bridge_error=not invalid and loop_error is None)
        summary["test_loop_error"]=loop_error
        summary["policy_standby_flow"]=args.policy_standby_flow
        summary["supermimic_checks"]=checks
        summary["error"]=None if all(checks.values()) else str(checks)
        summary["passed"]=all(checks.values())
    (args.output/"summary.json").write_text(json.dumps(summary, indent=2))
    if args.expect_policy_window_stop:
        print(json.dumps({"passed":summary["passed"],"checks":summary["supermimic_checks"]}),flush=True)
        if not summary["passed"]: raise SystemExit(1)
        return
    print(json.dumps(summary), flush=True)
    if args.fault:
        if reason or exit_code!=1 or invalid or not latest[0] or any(m.mode for m in latest[0].motor_cmd): raise SystemExit(1)
    elif reason or exit_code!=0 or any(summary["final_command_modes"] or [1]) or (
            summary["final_joint_error_rad"]>.15 if args.suspended else delta is None or (not args.zero_only and delta<.5)):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
