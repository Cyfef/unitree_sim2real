"""Standalone, CPU-only SuperMimic observation, motion and policy contract.

The formulas mirror supermimic_vibed/supermimic/{networks,motion,mdp,rotations}.py.
Keep this file versioned alongside the checkpoint; do not substitute the
unitree_rl_lab velocity policy's observation format.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.nn import functional as F

CONTROL_DT = 0.02
OBS_DIM = 99
JOINT_NAMES = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint",
    "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint",
    "right_ankle_pitch_joint", "right_ankle_roll_joint", "waist_yaw_joint",
    "waist_roll_joint", "waist_pitch_joint", "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint",
    "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

ARMATURE_5020 = 0.003609725
ARMATURE_7520_14 = 0.010177520
ARMATURE_7520_22 = 0.025101925
ARMATURE_4010 = 0.00425
MOTORS = {
    "hip_pitch": (88., 32., ARMATURE_7520_14),
    "hip_roll": (139., 20., ARMATURE_7520_22),
    "hip_yaw": (88., 32., ARMATURE_7520_14),
    "knee": (139., 20., ARMATURE_7520_22),
    "ankle_pitch": (50., 37., 2 * ARMATURE_5020),
    "ankle_roll": (50., 37., 2 * ARMATURE_5020),
    "waist_yaw": (88., 32., ARMATURE_7520_14),
    "waist_roll": (50., 37., 2 * ARMATURE_5020),
    "waist_pitch": (50., 37., 2 * ARMATURE_5020),
    "shoulder_pitch": (25., 37., ARMATURE_5020),
    "shoulder_roll": (25., 37., ARMATURE_5020),
    "shoulder_yaw": (25., 37., ARMATURE_5020),
    "elbow": (25., 37., ARMATURE_5020),
    "wrist_roll": (25., 37., ARMATURE_5020),
    "wrist_pitch": (5., 22., ARMATURE_4010),
    "wrist_yaw": (5., 22., ARMATURE_4010),
}


def motor(name: str) -> tuple[float, float, float]:
    return MOTORS[name.removeprefix("left_").removeprefix("right_").removesuffix("_joint")]


def default_q() -> np.ndarray:
    out = np.zeros(29, dtype=np.float32)
    for i, name in enumerate(JOINT_NAMES):
        if name.endswith("hip_pitch_joint"):
            out[i] = -0.312
        elif name.endswith("knee_joint"):
            out[i] = 0.669
        elif name.endswith("ankle_pitch_joint"):
            out[i] = -0.363
        elif name.endswith("elbow_joint"):
            out[i] = 0.6
        elif name == "left_shoulder_roll_joint":
            out[i] = 0.2
        elif name == "right_shoulder_roll_joint":
            out[i] = -0.2
        elif name.endswith("shoulder_pitch_joint"):
            out[i] = 0.2
    return out


def gains_and_limits() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    values = [motor(n) for n in JOINT_NAMES]
    effort = np.asarray([v[0] for v in values], dtype=np.float64)
    speed = np.asarray([v[1] for v in values], dtype=np.float64)
    armature = np.asarray([v[2] for v in values], dtype=np.float64)
    omega = 10 * 2.0 * 3.1415926535
    kp = armature * omega**2
    kd = 4.0 * armature * omega
    scale = 0.25 * effort / kp
    return kp, kd, scale, effort, speed


def quat_to_mat(q: torch.Tensor) -> torch.Tensor:
    q = F.normalize(q, dim=-1)
    w, x, y, z = q.unbind(-1)
    return torch.stack((
        1 - 2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
        2*(x*y+w*z), 1 - 2*(x*x+z*z), 2*(y*z-w*x),
        2*(x*z-w*y), 2*(y*z+w*x), 1 - 2*(x*x+y*y),
    ), -1).reshape(q.shape[:-1] + (3, 3))


def mat_to_quat(m: torch.Tensor) -> torch.Tensor:
    m00, m01, m02 = m[..., 0, 0], m[..., 0, 1], m[..., 0, 2]
    m10, m11, m12 = m[..., 1, 0], m[..., 1, 1], m[..., 1, 2]
    m20, m21, m22 = m[..., 2, 0], m[..., 2, 1], m[..., 2, 2]
    qw = torch.sqrt(torch.clamp(1+m00+m11+m22, min=1e-10))/2
    qx = torch.sqrt(torch.clamp(1+m00-m11-m22, min=1e-10))/2
    qy = torch.sqrt(torch.clamp(1-m00+m11-m22, min=1e-10))/2
    qz = torch.sqrt(torch.clamp(1-m00-m11+m22, min=1e-10))/2
    cands = torch.stack((
        torch.stack((qw, (m21-m12)/(4*qw), (m02-m20)/(4*qw), (m10-m01)/(4*qw)), -1),
        torch.stack(((m21-m12)/(4*qx), qx, (m01+m10)/(4*qx), (m02+m20)/(4*qx)), -1),
        torch.stack(((m02-m20)/(4*qy), (m01+m10)/(4*qy), qy, (m12+m21)/(4*qy)), -1),
        torch.stack(((m10-m01)/(4*qz), (m02+m20)/(4*qz), (m12+m21)/(4*qz), qz), -1),
    ), -2)
    idx = torch.stack((qw, qx, qy, qz), -1).argmax(-1)
    q = torch.gather(cands, -2, idx[..., None, None].expand(idx.shape+(1, 4))).squeeze(-2)
    q = F.normalize(q, dim=-1)
    return q * torch.where(q[..., :1] < 0, -1., 1.)


def quat_slerp(q0: torch.Tensor, q1: torch.Tensor, t: torch.Tensor) -> torch.Tensor:
    dot = (q0*q1).sum(-1, keepdim=True)
    q1 = torch.where(dot < 0, -q1, q1)
    dot = dot.abs().clamp(max=1.0)
    theta = torch.acos(dot)
    s = torch.sin(theta)
    t = t[..., None]
    w0 = torch.where(s < 1e-6, 1-t, torch.sin((1-t)*theta)/(s+1e-12))
    w1 = torch.where(s < 1e-6, t, torch.sin(t*theta)/(s+1e-12))
    return F.normalize(w0*q0+w1*q1, dim=-1)


def quat_to_rotvec(q: torch.Tensor) -> torch.Tensor:
    q = q * torch.where(q[..., :1] < 0, -1., 1.)
    v = q[..., 1:]
    sin_half = torch.linalg.norm(v, dim=-1, keepdim=True)
    half = torch.atan2(sin_half, q[..., :1])
    scale = torch.where(sin_half < 1e-6, 2/torch.clamp(q[..., :1], min=1e-6),
                        2*half/(sin_half+1e-12))
    return v*scale


def so3_log(R: torch.Tensor) -> torch.Tensor:
    return quat_to_rotvec(mat_to_quat(R))


def yaw_mat(R: torch.Tensor) -> torch.Tensor:
    fx, fy = R[..., 0, 0], R[..., 1, 0]
    n = torch.sqrt(fx*fx+fy*fy+1e-9)
    c, s = fx/n, fy/n
    o, one = torch.zeros_like(c), torch.ones_like(c)
    return torch.stack((c, -s, o, s, c, o, o, o, one), -1).reshape(R.shape[:-2]+(3, 3))


def mat_to_6d(R: torch.Tensor) -> torch.Tensor:
    return R[..., :, :2].reshape(R.shape[:-2]+(6,))


def gradient(x: torch.Tensor) -> torch.Tensor:
    g = torch.zeros_like(x)
    g[1:-1] = (x[2:]-x[:-2])/(2*CONTROL_DT)
    g[0] = (x[1]-x[0])/CONTROL_DT
    g[-1] = (x[-1]-x[-2])/CONTROL_DT
    return g


@dataclass
class Clip:
    name: str
    q: torch.Tensor
    qd: torch.Tensor
    R: torch.Tensor
    goals: torch.Tensor
    root_speed: torch.Tensor

    @property
    def frames(self) -> int:
        return int(self.q.shape[0])


def load_clip(path: Path, future: int) -> Clip:
    raw = torch.tensor(np.loadtxt(path, delimiter=","), dtype=torch.float32)
    if raw.ndim != 2 or raw.shape[1] != 36 or raw.shape[0] < 2:
        raise ValueError(f"invalid 36-column motion CSV: {path}")
    p, q_xyzw, j = raw[:, :3], raw[:, 3:7], raw[:, 7:36]
    quat = torch.cat((q_xyzw[:, 3:4], q_xyzw[:, :3]), -1)
    n = int(np.floor((raw.shape[0]-1)/30.0/CONTROL_DT))+1
    t = torch.arange(n, dtype=torch.float32)*CONTROL_DT*30.0
    i0 = torch.clamp(t.floor().long(), max=raw.shape[0]-2)
    a = (t-i0.float()).clamp(0, 1)
    p = p[i0]*(1-a[:, None])+p[i0+1]*a[:, None]
    j = j[i0]*(1-a[:, None])+j[i0+1]*a[:, None]
    quat = quat_slerp(quat[i0], quat[i0+1], a)
    R = quat_to_mat(quat)
    v = gradient(p)
    qd = gradient(j)
    w = torch.zeros_like(p)
    w[1:-1] = so3_log(R[2:]@R[:-2].transpose(-1, -2))/(2*CONTROL_DT)
    w[0] = so3_log(R[1]@R[0].T)/CONTROL_DT
    w[-1] = so3_log(R[-1]@R[-2].T)/CONTROL_DT
    Rt = R.transpose(-1, -2)
    v_local = (Rt@v.unsqueeze(-1)).squeeze(-1)
    w_local = (Rt@w.unsqueeze(-1)).squeeze(-1)
    idx = torch.arange(n)[:, None]+1+torch.arange(future)[None, :]
    idx = idx.clamp(max=n-1)
    Ht = yaw_mat(R).transpose(-1, -2)[:, None]
    dp = (Ht@(p[idx]-p[:, None]).unsqueeze(-1)).squeeze(-1)
    rot = mat_to_6d(Ht@R[idx])
    features = torch.cat((dp, rot, v_local[idx], w_local[idx], j[idx], qd[idx]*0.1), -1)
    if features.shape[-1] != 73:
        raise ValueError("motion goal contract changed")
    return Clip(path.stem, j, qd, R, features.reshape(n, -1), v)


class RunningNorm(nn.Module):
    def __init__(self, dim: int, eps: float = 1e-2):
        super().__init__()
        self.register_buffer("mean", torch.zeros(dim))
        self.register_buffer("var", torch.ones(dim))
        self.register_buffer("count", torch.zeros(()))
        self.eps = eps

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return (x-self.mean)/torch.sqrt(self.var+self.eps**2)


def mlp(inp: int, hidden: list[int], out: int) -> nn.Sequential:
    layers: list[nn.Module] = []
    d = inp
    for h in hidden:
        layers.extend((nn.Linear(d, h), nn.ELU()))
        d = h
    layers.append(nn.Linear(d, out))
    return nn.Sequential(*layers)


class Policy(nn.Module):
    def __init__(self, goal_frames: int, hist: int):
        super().__init__()
        self.enc = mlp(73*goal_frames, [2048, 1024, 512, 256], 32)
        self.dec = mlp(32+OBS_DIM*hist, [2048, 1024, 512, 256], 29)
        self.obs_norm = RunningNorm(OBS_DIM)
        self.goal_norm = RunningNorm(73*goal_frames)

    def forward(self, obs_hist: torch.Tensor, goal: torch.Tensor) -> torch.Tensor:
        z = torch.tanh(self.enc(self.goal_norm(goal)))
        o = self.obs_norm(obs_hist).flatten(-2)
        return self.dec(torch.cat((z, o), -1))


def observation(q: np.ndarray, dq: np.ndarray, gyro: np.ndarray,
                R: torch.Tensor, Rref: torch.Tensor, a_prev: torch.Tensor) -> torch.Tensor:
    q_t = torch.as_tensor(q, dtype=torch.float32)
    dq_t = torch.as_tensor(dq, dtype=torch.float32)
    gyro_t = torch.as_tensor(gyro, dtype=torch.float32)
    rel = mat_to_6d(R.T@Rref)
    obs = torch.cat((q_t-torch.as_tensor(default_q()), dq_t, gyro_t,
                     -R[2, :], a_prev, rel))
    if obs.shape != (OBS_DIM,) or not torch.isfinite(obs).all():
        raise ValueError("invalid policy observation")
    return obs


def load_policy(path: Path) -> tuple[Policy, int, int]:
    ckpt = torch.load(path, map_location="cpu", weights_only=False)
    args = ckpt["args"]
    hist, future = int(args["hist"]), int(args["goal_frames"])
    if not 1 <= hist <= 32 or not 1 <= future <= 32:
        raise ValueError("unsupported checkpoint history/future length")
    torch.set_num_threads(1)
    net = Policy(future, hist)
    net.load_state_dict(ckpt["pi"], strict=True)
    net.eval()
    return net, hist, future
