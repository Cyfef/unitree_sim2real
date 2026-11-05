"""Apply task-owned MuJoCo numerical settings to an already loaded model.

A task supplies the JSON profile. This module knows neither policy names nor
robot geometry, so other tasks can use the same application path.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import mujoco

JOINT_DOF_FIELDS = frozenset({'dof_armature', 'dof_damping', 'dof_frictionloss'})


def apply_profile(model, profile_path: Path) -> None:
    profile = json.loads(Path(profile_path).read_text())
    options = profile['options']
    joints = profile['joints']
    if not isinstance(options, dict) or not isinstance(joints, dict):
        raise ValueError('physics profile requires options and joints objects')
    for field, value in options.items():
        if field.startswith('_') or not hasattr(model.opt, field):
            raise ValueError(f'unknown MuJoCo option: {field}')
        current = getattr(model.opt, field)
        if isinstance(current, np.ndarray):
            replacement = np.asarray(value)
            if replacement.shape != current.shape or not np.isfinite(replacement).all():
                raise ValueError(f'invalid MuJoCo option array: {field}')
            current[:] = replacement
        elif isinstance(current, (int, float)) and isinstance(value, (int, float)) and np.isfinite(value):
            setattr(model.opt, field, value)
        else:
            raise ValueError(f'invalid MuJoCo option value: {field}')
    for name, settings in joints.items():
        joint = model.joint(name)
        if int(model.jnt_type[joint.id]) not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
            raise ValueError(f'expected one-DoF joint: {name}')
        if not isinstance(settings, dict) or set(settings) - JOINT_DOF_FIELDS:
            raise ValueError(f'invalid DoF settings for {name}')
        dof = model.jnt_dofadr[joint.id]
        for field, value in settings.items():
            if not isinstance(value, (int, float)) or not np.isfinite(value):
                raise ValueError(f'invalid {field} for {name}')
            getattr(model, field)[dof] = value
