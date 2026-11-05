#!/usr/bin/env python3
"""Read G1 joint state through SDK2. This program never publishes commands."""

import argparse
import threading
import time

JOINTS = (
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll", "right_hip_pitch", "right_hip_roll",
    "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch", "left_shoulder_pitch",
    "left_shoulder_roll", "left_shoulder_yaw", "left_elbow", "left_wrist_roll",
    "left_wrist_pitch", "left_wrist_yaw", "right_shoulder_pitch",
    "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
)


def main():
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--network", default="eno1", help="DDS network interface")
    parser.add_argument("--joint", choices=JOINTS, help="show just this joint")
    parser.add_argument("--watch", action="store_true", help="print once per second")
    args = parser.parse_args()

    lock = threading.Lock()
    ready = threading.Event()
    latest = [None, 0.0]

    def on_state(msg):
        with lock:
            latest[:] = [msg, time.monotonic()]
        ready.set()

    ChannelFactoryInitialize(0, args.network)
    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(on_state, 1)
    if not ready.wait(5):
        parser.exit(1, "No low-state message received within 5 seconds.\n")
    try:
        while True:
            with lock:
                msg, received = latest
            age = time.monotonic() - received
            if age > 1:
                parser.exit(1, f"Low-state stream is stale ({age:.2f} s).\n")
            print(f"mode_machine={msg.mode_machine} mode_pr={msg.mode_pr} age={age:.3f}s")
            for index, name in enumerate(JOINTS):
                if args.joint is None or args.joint == name:
                    motor = msg.motor_state[index]
                    print(f"{index:2d} {name:22s} q={motor.q:+.4f} rad "
                          f"dq={motor.dq:+.4f} rad/s mode={motor.mode} "
                          f"tau_est={motor.tau_est:+.3f} Nm error={motor.motorstate}")
            if not args.watch:
                break
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
