#!/usr/bin/env python3
"""Finite, read-only G1 recorder. Never creates a publisher or sends commands."""
import argparse
import csv
import json
import math
from pathlib import Path
import signal
import struct
import threading
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_, HandState_
from unitree_sdk2py.utils.crc import CRC


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--network',default='eno1')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--rate',type=int,choices=(20,50,100),default=100)
    p.add_argument('--duration',type=float,default=600.)
    args=p.parse_args()
    if not 1<=args.duration<=1800:p.error('duration must be in [1,1800] seconds')
    if args.output.exists() or args.output.with_suffix('.json').exists():p.error('output already exists')
    ChannelFactoryInitialize(0,args.network)
    lock=threading.Lock();crc=CRC();latest=[None,0.];command=[None,0.]
    hands={};bad=[0,0];stopped=threading.Event();subs=[]
    def on_state(s):
        valid=crc.Crc(s)==s.crc
        with lock:latest[:]=[s,time.monotonic()];bad[0]+=not valid
    def on_command(c):
        valid=crc.Crc(c)==c.crc
        with lock:command[:]=[c,time.monotonic()];bad[1]+=not valid
    def on_hand(h,side):
        with lock:hands[side]=([m.q for m in h.motor_state[:7]],time.monotonic())
    for topic,kind,callback in [('rt/lowstate',LowState_,on_state),('rt/lowcmd',LowCmd_,on_command)]:
        sub=ChannelSubscriber(topic,kind);sub.Init(callback,1);subs.append(sub)
    for side in ('left','right'):
        sub=ChannelSubscriber(f'rt/dex3/{side}/state',HandState_)
        sub.Init(lambda h,side=side:on_hand(h,side),1);subs.append(sub)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:stopped.set())
    header=['t','wall_time','state_age','command_age','tick','bad_state_crc','bad_command_crc']
    header += [f'{kind}_{axis}' for kind,axes in [('quat','wxyz'),('gyro','xyz'),('accel_sdk','xyz')] for axis in axes]
    fields=('q','dq','tau','feedback_mode','motor_error','command_q','command_dq','command_tau','kp','kd','command_mode')
    header += [f'{field}_{i}' for field in fields for i in range(29)]
    header += ['all35_outgoing_disabled','remote_buttons','ly','max_temperature']
    header += [f'{side}_hand_{i}' for side in ('left','right') for i in range(7)]
    start=time.monotonic();next_sample=start;report=0.;active=False;disabled_since=None;count=0
    reason='duration';worst_age=0.
    with args.output.open('x',newline='') as f:
        writer=csv.writer(f);writer.writerow(header)
        while not stopped.is_set() and time.monotonic()-start<args.duration:
            with lock:s,stamp=latest;c,cstamp=command;errors=bad.copy();hs=hands.copy()
            now=time.monotonic();t=now-start
            if s is None:
                if t>5:reason='no low state';break
                time.sleep(.01);continue
            worst_age=max(worst_age,now-stamp)
            all_disabled=c is not None and all(m.mode==0 for m in c.motor_cmd)
            if c and any(m.kp>0 for m in c.motor_cmd[:29]):active=True
            values=[t,time.time(),now-stamp,now-cstamp if c else math.nan,s.tick,*errors]
            values+=list(s.imu_state.quaternion)+list(s.imu_state.gyroscope)+list(s.imu_state.accelerometer)
            feedback=s.motor_state[:29]
            values += [m.q for m in feedback]+[m.dq for m in feedback]+[m.tau_est for m in feedback]
            values += [m.mode for m in feedback]+[m.motorstate for m in feedback]
            for field in ('q','dq','tau','kp','kd','mode'):
                values += [getattr(m,field) for m in c.motor_cmd[:29]] if c else [math.nan]*29
            remote=bytes(s.wireless_remote);ly=struct.unpack('<f',remote[20:24])[0]
            values += [int(all_disabled),int.from_bytes(remote[2:4],'little'),ly,max(max(m.temperature) for m in feedback)]
            for side in ('left','right'):
                q,when=hs.get(side,([math.nan]*7,0.))
                values+=q if now-when<.1 else [math.nan]*7
            assert len(values)==len(header)
            writer.writerow(values);count+=1
            if t>=report:
                quat=s.imu_state.quaternion;tilt=math.degrees(math.acos(max(-1.,min(1.,1-2*(quat[1]**2+quat[2]**2)))))
                wrist=max(abs(feedback[i].tau_est) for i in (20,21,27,28))
                print(f't={t:.1f}s age={now-stamp:.3f}s tilt={tilt:.2f}deg wrist={wrist:.2f}Nm ly={ly:.3f} crc={errors} disabled={all_disabled}',flush=True)
                f.flush();report=t+2.
            if active and all_disabled:
                if disabled_since is None:disabled_since=now
                if now-disabled_since>.5:reason='all35 outgoing disabled';break
            else:disabled_since=None
            next_sample=max(next_sample+1/args.rate,time.monotonic())
            stopped.wait(max(0,next_sample-time.monotonic()))
    if stopped.is_set():reason='signal'
    for sub in subs:sub.Close()
    args.output.with_suffix('.json').write_text(json.dumps({'read_only':True,'rate_hz':args.rate,
        'rows':count,'elapsed_s':time.monotonic()-start,'stop_reason':reason,'crc_errors':bad,
        'max_recorded_state_age_s':worst_age,'position_command_seen':active,
        'acceleration':'Unmodified SDK accelerometer values; bias/gravity compensation not applied.'},indent=2)+'\n')
    print('Recorder finished:',reason,flush=True)
    if reason=='no low state':raise SystemExit(1)


if __name__=='__main__':main()
