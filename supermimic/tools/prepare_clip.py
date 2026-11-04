"""Prepare one bounded SuperMimic motion for the copied RL Lab controller.

No DDS import, no robot command. This writes a candidate, not an approved launch.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import struct
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from contract import JOINT_NAMES, load_clip  # noqa: E402


def model_limits(model_path: Path) -> tuple[np.ndarray, np.ndarray]:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    lo, hi = [], []
    for name in JOINT_NAMES:
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint < 0 or not model.jnt_limited[joint]:
            raise ValueError(f"joint missing/unlimited: {name}")
        lo.append(model.jnt_range[joint, 0])
        hi.append(model.jnt_range[joint, 1])
    return np.asarray(lo), np.asarray(hi)


def grounded_height(model_path: Path, ref: dict) -> float:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    data.qpos[2] = 0
    data.qpos[3:7] = ref["pelvis_quaternion"]
    for name, value in zip(JOINT_NAMES, ref["q"]):
        joint = model.joint(name)
        data.qpos[model.jnt_qposadr[joint.id]] = value
    fingers = ("thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1")
    for side, values in ref["hands"].items():
        for name, value in zip(fingers, values):
            joint = model.joint(f"{side}_hand_{name}_joint")
            data.qpos[model.jnt_qposadr[joint.id]] = value
    mujoco.mj_forward(model, data)
    bottoms = []
    for side in ("left", "right"):
        body = model.body(side + "_ankle_roll_link").id
        bottoms.extend(float(data.geom_xpos[i, 2] - model.geom_size[i, 0])
                       for i in range(model.ngeom) if model.geom_bodyid[i] == body
                       and model.geom_type[i] == mujoco.mjtGeom.mjGEOM_SPHERE)
    if len(bottoms) != 8:
        raise ValueError("expected eight foot contact spheres")
    height = -min(bottoms)
    if not .65 < height < .9:
        raise ValueError(f"unsupported inferred grounded height {height:.3f} m")
    return height


def candidate_frames(clip, lo: np.ndarray, hi: np.ndarray, search_seconds: float) -> list[tuple[float, float]]:
    out = []
    for i in range(min(clip.frames - 1, round(search_seconds / .02))):
        q = clip.q[i].numpy()
        qd = float(clip.qd[i].abs().max())
        tilt = math.acos(float(np.clip(clip.R[i, 2, 2], -1, 1)))
        speed = float(clip.root_speed[i].norm())
        if qd <= 1 and tilt <= .35 and speed <= .5 and np.all(q >= lo + .05) and np.all(q <= hi - .05):
            out.append((round(i * .02, 2), round(qd, 3)))
    return out


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--clip", required=True)
    p.add_argument("--policy", choices=("baseline", "backpack"), required=True)
    p.add_argument("--start-seconds", type=float, default=0.0)
    p.add_argument("--max-seconds", type=float, default=5.0)
    p.add_argument("--full-clip-simulation", action="store_true", help="entire clip, isolated simulation only")
    p.add_argument("--suggest-starts", action="store_true")
    p.add_argument("--reference", type=Path, help="fresh disabled-pose capture")
    p.add_argument("--output-dir", type=Path)
    p.add_argument("--model", type=Path, default=ROOT / "assets/model/g1_29dof_with_hand_rev_1_0.xml")
    args = p.parse_args()
    motion = ROOT / "assets/motions" / (args.clip + ".csv")
    if not motion.is_file() or motion.stem != args.clip:
        p.error("unknown bundled clip")
    if not args.model.is_file():
        p.error("bundled G1 model missing")
    clip = load_clip(motion, 10)
    lo, hi = model_limits(args.model)
    if args.suggest_starts:
        starts = candidate_frames(clip, lo, hi, 30)
        print(json.dumps({"clip": args.clip, "duration_s": round((clip.frames - 1) * .02, 2),
                          "first_30s_admissible_frames": starts[:40], "count": len(starts)}, indent=2))
        return
    if args.reference is None or args.output_dir is None or args.start_seconds is None:
        p.error("preparation requires --reference, --output-dir and --start-seconds")
    if not math.isfinite(args.max_seconds) or not .5 <= args.max_seconds <= 30:
        p.error("initial test window must be 0.5-30 seconds")
    if not math.isfinite(args.start_seconds) or args.start_seconds < 0:
        p.error("invalid start time")
    start = round(args.start_seconds / .02)
    stop = start + math.ceil(args.max_seconds / .02) + 1
    if args.full_clip_simulation:
        if start != 0:
            p.error("full-clip simulation must start at frame zero")
        stop = clip.frames
        args.max_seconds = (stop - start - 1) * .02
        if args.max_seconds > 600:
            p.error("full-clip simulation is limited to 600 seconds")
    if start >= clip.frames - 1 or stop > clip.frames:
        p.error("motion window runs past the end of the clip")
    q0 = clip.q[start].numpy()
    qd0 = float(clip.qd[start].abs().max())
    speed0 = float(clip.root_speed[start].norm())
    tilt0 = math.acos(float(np.clip(clip.R[start, 2, 2], -1, 1)))
    if speed0 > .5 or tilt0 > .35 or np.any(q0 < lo + .05) or np.any(q0 > hi - .05):
        raise ValueError(f"unsafe motion entry at {start*.02:.2f}s: max_qd={qd0:.3f}, "
                         f"root_speed={speed0:.3f}, tilt={tilt0:.3f}")
    ref = json.loads(args.reference.read_text())
    if len(ref.get("q", [])) != 29 or set(ref.get("hands", {})) != {"left", "right"}:
        raise ValueError("invalid captured reference")
    qref = np.asarray(ref["q"], dtype=float)
    if not np.isfinite(qref).all() or np.any(qref < lo) or np.any(qref > hi):
        raise ValueError("reference pose outside model range")
    height = grounded_height(args.model, ref)

    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(args.reference, out / "reference.json")
    binary = out / "motion.bin"
    with binary.open("wb") as stream:
        stream.write(struct.pack("<4sII", b"SMC1", stop - start, 730))
        for i in range(start, stop):
            for tensor in (clip.q[i], clip.R[i].reshape(-1), clip.goals[i]):
                stream.write(tensor.numpy().astype("<f4", copy=False).tobytes())

    template = json.loads((ROOT / "rl_lab_controller/g1_29dof/config/recovery_template.json").read_text())
    cfg = template
    qclear = qref.copy()
    qclear[16], qclear[23] = .45, -.45
    stand = cfg["FSM"]["FixStand"]
    clearance_stand = np.asarray(stand["qs"][-2], dtype=float)
    nominal = np.asarray(stand["qs"][-1], dtype=float)
    # Preserve all stages of the proven recovery, then align to the clip.
    # The controller requires a stable nominal posture before this last stage.
    from stand_plan import alignment_waypoints
    alignment,alignment_report=alignment_waypoints(args.model,args.reference,ref,nominal,q0)
    waypoints = [qref, qclear, clearance_stand, nominal]+alignment[1:]
    for waypoint in waypoints:
        if np.any(waypoint < lo) or np.any(waypoint > hi):
            raise ValueError("recovery waypoint outside model limits")
    times = [0.]
    for k,(a,b) in enumerate(zip(waypoints,waypoints[1:])):
        minimum=(3.,9.,4.)[k] if k<3 else 4.
        # Original RL Lab recovery is linear; alignment uses a half-cosine.
        speed=.15 if k<3 else .10
        slope_factor=1. if k<3 else math.pi/2
        times.append(times[-1] + max(minimum, slope_factor * float(np.max(np.abs(b - a))) / speed))
    from stand_plan import plan_stand
    dense_t, dense_q, dense_tau, stand_caps, gravity_peaks = plan_stand(
        args.model, args.reference, ref, waypoints, times,linear_segments=3)
    cfg["FSM"]["FixStand"]["qs"] = dense_q
    cfg["FSM"]["FixStand"]["ts"] = dense_t
    cfg["FSM"]["FixStand"]["taus"] = dense_tau
    cfg["FSM"]["FixStand"].update(
        nominal_time=times[3], pace_recovery=False, tracking_pause_error=.12,
        tracking_resume_error=.08, tracking_pause_timeout=3.,
        nominal_settle_timeout=5., nominal_hold_seconds=.3)
    cfg["FSM"]["FixStand"]["transitions"]["Velocity"] = "RB + A.on_pressed"
    cfg["FSM"]["_"]["Velocity"]["type"] = "SuperMimic"
    controller_root = ROOT / "rl_lab_controller/g1_29dof"
    def relative(path: Path) -> str:
        return os.path.relpath(path.resolve(), controller_root.resolve())
    cfg["FSM"]["Velocity"] = {
        "transitions": {"Passive": "LT + B.on_pressed"},
        "policy_file": relative(ROOT / "assets/policies" / args.policy / "policy.onnx"),
        "meta_file": relative(ROOT / "assets/policies" / args.policy / "policy_meta.json"),
        "motion_file": relative(binary),
        "max_seconds": args.max_seconds,
        "simulation_only": args.full_clip_simulation,
        "transition_seconds": 1.0,
        "standby_flow": True,
    }
    cfg["Safety"] = {"reference": relative(out / "reference.json"),
                     "stand_torque_limits": stand_caps,
                     "trace_path": relative(out / "safety_trace.csv")}
    cfg["Startup"] = {"auto_stand": True, "stand_delay_seconds": 3}
    (out / "candidate.json").write_text(json.dumps(cfg, indent=2) + "\n")
    report = {"policy": args.policy, "clip": args.clip, "start_seconds": start * .02,
              "max_seconds": args.max_seconds,
        "simulation_only": args.full_clip_simulation, "entry_max_qd": qd0,
              "entry_root_speed": speed0, "entry_tilt_rad": tilt0,
              "start_pose_delta_rad": float(np.max(np.abs(q0 - qref))),
              "stand_duration_seconds": times[-1],
              "nominal_stand_seconds": times[3],
              "recovery_flow": "rl_lab_linear_then_settle",
              "first_frame_alignment_seconds": times[-1]-times[3],
              "alignment_route": alignment_report,
              "stand_progress_allowance_seconds": 30.,
              "posture_model": str(args.model.resolve()),
              "assumed_backpack_mass_kg": 1.9,
              "inferred_grounded_root_height_m": height,
              "status": "candidate_not_validated",
              "standby_flow": True, "arm_gravity_peak_Nm": gravity_peaks[15:],
              "stand_torque_limits_Nm": stand_caps,
              "reference_entry_velocity_is_dynamic": qd0 > 1}
    (out / "preparation.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
