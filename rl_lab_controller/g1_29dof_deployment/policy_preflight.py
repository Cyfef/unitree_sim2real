#!/usr/bin/env python3
"""Read-only checks before launching the G1 controller with a validated pose."""
import argparse
import json
import math
from pathlib import Path
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_, HandState_
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
from unitree_sdk2py.utils.crc import CRC


def inspect_robot(network, ref=None):
    """Read-only inspection; without a reference, capture a fresh disabled pose."""
    if ref is not None and (len(ref["q"])!=29 or set(ref["hands"])!={"left","right"}):
        raise SystemExit("invalid reference")
    ChannelFactoryInitialize(0,network)
    switch=MotionSwitcherClient();switch.SetTimeout(5);switch.Init()
    code,result=switch.CheckMode()
    if code or not isinstance(result,dict) or result.get("name")!="":
        raise SystemExit(f"active/unknown motion mode: {code}, {result}")
    services=RobotStateClient();services.SetTimeout(5);services.Init()
    code,rows=services.ServiceList()
    if code: code,rows=services.ServiceList()
    if code or rows is None: raise SystemExit(f"cannot inspect services: {code}")
    status={r.name:r.status for r in rows}
    for name in ("ai_sport","auto_test_low","auto_test_arm","g1_arm_example"):
        if status.get(name)!=1: raise SystemExit(f"service must be stopped: {name}, status={status.get(name)}")
    states=[];commands=[];hands={};hand_times={};hand_samples={"left":[],"right":[]};invalid=[];subs=[];crc=CRC();last=[0.]
    def on_state(s):
        if crc.Crc(s)!=s.crc: invalid.append("CRC")
        states.append(s);last[0]=time.monotonic()
    for topic,cls,callback in (("rt/lowstate",LowState_,on_state),("rt/lowcmd",LowCmd_,lambda _:commands.append(1))):
        sub=ChannelSubscriber(topic,cls);sub.Init(callback,1);subs.append(sub)
    def on_hand(h,side):
        hands[side]=[m.q for m in h.motor_state[:7]];hand_times[side]=time.monotonic()
        hand_samples[side].append(hands[side])
    for side in ("left","right"):
        sub=ChannelSubscriber(f"rt/dex3/{side}/state",HandState_)
        sub.Init(lambda h,side=side:on_hand(h,side),1);subs.append(sub)
    time.sleep(3)
    if len(states)<100 or time.monotonic()-last[0]>.05 or invalid or commands:
        raise SystemExit(f"invalid/stale state or competing commands: samples={len(states)}, bad={invalid[:1]}, commands={len(commands)}")
    state=states[-1]
    if state.mode_machine!=5 or state.mode_pr!=0: raise SystemExit("unexpected machine/ankle mode")
    maxdiff=0.
    reference_q=ref["q"] if ref is not None else [m.q for m in state.motor_state[:29]]
    for i,q in enumerate(reference_q):
        m=state.motor_state[i]
        if m.motorstate or m.mode not in (0,1) or not all(math.isfinite(v) for v in (m.q,m.dq,m.tau_est,q)):
            raise SystemExit(f"joint {i}: invalid state")
        if ref is None and m.mode!=0:
            raise SystemExit(f"joint {i}: capture requires disabled motors")
        if abs(m.dq)>.1 or max(s.motor_state[i].q for s in states)-min(s.motor_state[i].q for s in states)>.005:
            raise SystemExit(f"joint {i}: robot must be stationary")
        maxdiff=max(maxdiff,abs(m.q-q))
    if maxdiff>.02:
        i=max(range(29),key=lambda j:abs(state.motor_state[j].q-reference_q[j]))
        from joint_state import JOINTS
        raise SystemExit(f"body pose differs from validated reference by {maxdiff:.4f} rad "
                         f"({math.degrees(maxdiff):.1f} deg) at {JOINTS[i]} [{i}]; "
                         "prepare a new session from the current pose with policy_prepare.py")
    if set(hands)!={"left","right"} or any(time.monotonic()-t>.1 for t in hand_times.values()):
        raise SystemExit("missing/stale hand states")
    for side in hands:
        if not all(len(sample)==7 and all(math.isfinite(q) for q in sample) for sample in hand_samples[side]):
            raise SystemExit(f"{side} hand: invalid samples")
        if any(max(sample[i] for sample in hand_samples[side])-min(sample[i] for sample in hand_samples[side])>.03 for i in range(7)):
            raise SystemExit(f"{side} hand must be stationary")
        if ref is None:
            if len(hands[side])!=7 or not all(math.isfinite(q) for q in hands[side]):
                raise SystemExit(f"{side} hand: invalid pose")
            continue
        if len(hands[side])!=7 or len(ref["hands"][side])!=7 or any(
                not math.isfinite(q) or abs(q-r)>.03 for q,r in zip(hands[side],ref["hands"][side])):
            raise SystemExit(f"{side} hand pose changed")
    quat=state.imu_state.quaternion
    if not all(math.isfinite(q) for q in quat) or not .98<sum(q*q for q in quat)<1.02:
        raise SystemExit("invalid pelvis orientation")
    # Compare gravity directions; yaw rotation of the suspension is harmless.
    def down(q):
        w,x,y,z=q
        return (2*(w*y-x*z),-2*(w*x+y*z),-1+2*(x*x+y*y))
    if ref is not None and math.sqrt(sum((a-b)**2 for a,b in zip(down(quat),down(ref["pelvis_quaternion"]))))>.1:
        raise SystemExit("pelvis tilt differs from validated reference")
    if any(not math.isfinite(v) or abs(v)>.1 for v in state.imu_state.gyroscope):
        raise SystemExit("suspension is moving")
    result={"q":[m.q for m in state.motor_state[:29]], "hands":hands,
            "pelvis_quaternion":list(quat), "captured_wall_time":time.time()}
    for sub in subs:
        sub.Close()
    print(f"Preflight passed: stationary, CRC valid, no competing controller, body difference {maxdiff:.5f} rad.")
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--network",default="eno1")
    p.add_argument("--reference",type=Path,required=True)
    args=p.parse_args()
    inspect_robot(args.network,json.loads(args.reference.read_text()))


if __name__=="__main__": main()
