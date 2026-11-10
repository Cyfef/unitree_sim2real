#!/usr/bin/env python3
"""Render a recorded SuperMimic simulation pose trace to an MP4, after physics ends."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')

import argparse
import json
from pathlib import Path
import sys

import imageio_ffmpeg
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'tools')]
from model import load_model


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='completed run directory containing simulation/video_states.npz')
    args = parser.parse_args()
    run = args.run.resolve()
    trace_path = run / 'simulation/video_states.npz'
    output = run / 'simulation/video.mp4'
    if output.exists():
        parser.error(f'video already exists: {output}')
    if not trace_path.is_file():
        parser.error(f'no policy-motion video trace: {trace_path}')
    with np.load(trace_path, allow_pickle=False) as trace:
        times = trace['time_s']
        poses = trace['qpos']
    if poses.ndim != 2 or len(poses) == 0 or times.shape != (len(poses),):
        raise ValueError('invalid video pose trace')
    if not np.all(np.isfinite(poses)) or not np.all(np.isfinite(times)) or np.any(np.diff(times) <= 0):
        raise ValueError('nonfinite or unordered video poses')

    reference_path = run / 'reference.json'
    reference = json.loads(reference_path.read_text())
    model_path = ROOT.parent / 'models/g1_29dof_with_hand_rev_1_0.xml'
    model = load_model(model_path.resolve(), str(reference_path), False, reference, 1.9)
    if poses.shape[1] != model.nq:
        raise ValueError(f'video pose width {poses.shape[1]} does not match model nq {model.nq}')
    data = mujoco.MjData(model)
    pelvis = model.body('pelvis').id
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance = 3.0
    camera.azimuth = 135
    camera.elevation = -18
    width, height, fps = 640, 480, 20
    temporary = output.with_name('video.partial.mp4')
    temporary.unlink(missing_ok=True)
    writer = imageio_ffmpeg.write_frames(str(temporary), (width, height), fps=fps,
                                         codec='libx264', quality=7, pix_fmt_out='yuv420p')
    try:
        writer.send(None)
        with mujoco.Renderer(model, width=width, height=height) as renderer:
            for pose in poses:
                data.qpos[:] = pose
                mujoco.mj_forward(model, data)
                camera.lookat[:] = data.xpos[pelvis] + np.array([0., 0., .15])
                renderer.update_scene(data, camera=camera)
                writer.send(renderer.render().tobytes())
        writer.close()
        temporary.replace(output)
    except BaseException:
        writer.close()
        temporary.unlink(missing_ok=True)
        raise
    print(json.dumps({'video': str(output), 'frames': len(poses),
                      'fps': fps, 'duration_s': len(poses) / fps}))


if __name__ == '__main__':
    main()
