#!/usr/bin/env python3
"""Release known official control, then disable motors under full hoist support."""
import argparse
import json
import math
from pathlib import Path
import signal
import threading
import time

SERVICES = ('ai_sport', 'auto_test_low', 'auto_test_arm', 'g1_arm_example')


def check_state(state, age, bad_crc, require_still=True):
    if state is None or age > .05 or bad_crc:
        raise RuntimeError('missing/stale state or CRC error')
    if state.mode_machine != 5 or state.mode_pr != 0:
        raise RuntimeError('unexpected robot/ankle mode')
    q = state.imu_state.quaternion
    if len(q) != 4 or not all(math.isfinite(v) for v in q) or not .98 < sum(v*v for v in q) < 1.02:
        raise RuntimeError('invalid IMU quaternion')
    for i, m in enumerate(state.motor_state[:29]):
        if m.motorstate or m.mode not in (0, 1) or not all(math.isfinite(v) for v in (m.q,m.dq,m.tau_est)):
            raise RuntimeError(f'joint {i}: invalid/error state')
        if require_still and abs(m.dq) > .1:
            raise RuntimeError(f'joint {i}: moving; wait for full hoist support and stillness')


def disabled_command(factory, crc):
    cmd = factory()
    cmd.mode_machine = 5
    cmd.mode_pr = 0
    for m in cmd.motor_cmd:
        m.mode = 0
        m.q = m.dq = m.kp = m.kd = m.tau = 0.
    cmd.crc = crc.Crc(cmd)
    return cmd


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--network', default='eno1')
    p.add_argument('--supported-grounded', action='store_true')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not args.supported_grounded:
        p.error('requires full hoist support, grounded feet, attended emergency stop and clear joints')
    if args.output.exists():
        p.error('output already exists')
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    from unitree_sdk2py.go2.robot_state.robot_state_client import RobotStateClient
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
    from unitree_sdk2py.utils.crc import CRC

    report = {'status':'starting','full_support_asserted':True,'disable_packets_sent':0}
    subs = []
    publisher = None
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.set())
    try:
        ChannelFactoryInitialize(0,args.network)
        crc = CRC()
        lock = threading.Lock()
        latest = [None,0.]
        last_command = [0.]
        bad_crc = [0]
        conflict = threading.Event()
        publishing = threading.Event()
        def on_state(s):
            with lock:
                latest[:] = [s,time.monotonic()]
                bad_crc[0] += crc.Crc(s) != s.crc
        def on_command(c):
            with lock:
                last_command[0] = time.monotonic()
            if publishing.is_set() and (crc.Crc(c) != c.crc or any(
                    m.mode != 0 or any(v != 0 for v in (m.kp,m.kd,m.tau,m.dq)) for m in c.motor_cmd)):
                conflict.set()
        for topic, kind, callback in [('rt/lowstate',LowState_,on_state),('rt/lowcmd',LowCmd_,on_command)]:
            sub = ChannelSubscriber(topic,kind)
            sub.Init(callback,1)
            subs.append(sub)
        # Validate live state before changing the official controller.
        if stopped.wait(1):
            raise RuntimeError('interrupted')
        with lock:
            check_state(latest[0],time.monotonic()-latest[1],bad_crc[0])
        motion = MotionSwitcherClient();motion.SetTimeout(3);motion.Init()
        services = RobotStateClient();services.SetTimeout(3);services.Init()
        code, mode = motion.CheckMode()
        if code or not isinstance(mode,dict):
            raise RuntimeError(f'cannot query motion mode: {code}, {mode}')
        if mode.get('name'):
            code, _ = motion.ReleaseMode()
            print('Release official mode:',code,flush=True)
            if code:
                raise RuntimeError(f'ReleaseMode failed: {code}')
        code, rows = services.ServiceList()
        if code or rows is None:
            raise RuntimeError(f'ServiceList failed: {code}')
        statuses = {r.name:r.status for r in rows}
        for name in SERVICES:
            if statuses.get(name) not in (0,1):
                raise RuntimeError(f'unknown service status: {name}={statuses.get(name)}')
            if statuses[name] == 0:
                code = services.ServiceSwitch(name,False)
                print(f'Stop {name}: {code}',flush=True)
                if code:
                    raise RuntimeError(f'cannot stop {name}: {code}; may be protected')
        deadline = time.monotonic()+20
        while True:
            if stopped.wait(.5):
                raise RuntimeError('interrupted')
            code, mode = motion.CheckMode()
            scode, rows = services.ServiceList()
            statuses = {r.name:r.status for r in rows} if rows is not None else {}
            if code == 0 and isinstance(mode,dict) and mode.get('name') == '' and scode == 0 and all(statuses.get(n)==1 for n in SERVICES):
                break
            if time.monotonic() > deadline:
                raise RuntimeError(f'official control did not stop: {code}, {mode}, {statuses}')
        report.update(official_mode=mode,services=statuses)
        # Refuse to disable alongside an unknown publisher, even after official services stop.
        print('Official control stopped. Checking command channel and stillness...',flush=True)
        if stopped.wait(3):
            raise RuntimeError('interrupted')
        with lock:
            if last_command[0] and time.monotonic()-last_command[0] < 2.5:
                raise RuntimeError('another lowcmd publisher remains; stop it yourself; no disable packets sent')
            check_state(latest[0],time.monotonic()-latest[1],bad_crc[0])
            enabled = [i for i,m in enumerate(latest[0].motor_state[:29]) if m.mode != 0]
        report['enabled_joints_before'] = enabled
        if enabled:
            print('Sending disabled commands only (all 35 motors, zero gains/torque); limbs may relax.',flush=True)
            publisher = ChannelPublisher('rt/lowcmd',LowCmd_)
            publisher.Init()
            cmd = disabled_command(unitree_hg_msg_dds__LowCmd_,crc)
            publishing.set()
            for _ in range(200):
                if stopped.is_set() or conflict.is_set():
                    raise RuntimeError('interrupted or competing command appeared; disable publisher stopped')
                with lock:
                    check_state(latest[0],time.monotonic()-latest[1],bad_crc[0],require_still=False)
                if publisher.Write(cmd) is False:
                    raise RuntimeError('disable command publish failed')
                report['disable_packets_sent'] += 1
                stopped.wait(.005)
            publisher.Close();publisher=None
            publishing.clear()
        deadline = time.monotonic()+20
        while True:
            if stopped.wait(.5):
                raise RuntimeError('interrupted')
            with lock:
                state = latest[0]
                check_state(state,time.monotonic()-latest[1],bad_crc[0],require_still=False)
                still = all(abs(m.dq)<=.1 for m in state.motor_state[:29])
                enabled = [i for i,m in enumerate(state.motor_state[:29]) if m.mode != 0]
            if not enabled and still:
                break
            if time.monotonic() > deadline:
                raise RuntimeError(f'feedback not ready: enabled joints {enabled}, stationary={still}; do not bypass capture check')
        if stopped.wait(3):
            raise RuntimeError('interrupted')
        with lock:
            check_state(latest[0],time.monotonic()-latest[1],bad_crc[0])
            if last_command[0] and time.monotonic()-last_command[0] < 2.5:
                raise RuntimeError('lowcmd publisher remains after disable; abort')
        report['status'] = 'released_and_disabled'
        print('Official control stopped, motor feedback disabled, command channel quiet.',flush=True)
    except BaseException as exc:
        report.update(status='failed',error=str(exc) or type(exc).__name__)
        raise
    finally:
        if publisher is not None:
            publisher.Close()
        for sub in subs:
            sub.Close()
        args.output.write_text(json.dumps(report,indent=2)+'\n')


if __name__ == '__main__':
    main()
