"""Thread bench for eyebolt-lift (not a benchmark task): the eyebolt's thread with a fixed start, under the task's
physics (240 Hz, CPU). The arms stay in their ready pose.

A housing block with three tapped M10 holes; an eyebolt standing loose in each, and one in a tray. A "virtual hand"
turns an eyebolt: a torque about its axis from a torque-limited speed controller (1 turn per second clockwise), while
holding it upright and pressing it down lightly until it catches; then pressing 10 N.

    python el_bench_thread.py [--out bench_runs] [--device cpu]     # writes el_bench_thread.json, prints BENCH {...}

Cases (the pull direction is world +x for all holes; each hole's thread is laid out so the eye stops 75 degrees short
of it at shoulder contact):
- e1: starts at yaw 0; turned in until the shoulder sits (gap < 0.5 mm, clamp holding), then further until the eye
  is in line with the pull; let go: does it stay?
- e2: the same, starting at yaw 50 degrees: the eye must stop at the same angle at shoulder contact as e1's.
- e3: turned in until seated, then turned back out until loose; lifted gently: does it come out?
- e4: standing in the tray, untouched: does it stand still?
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

TASK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TASK_DIR))
import el_parts as el  # noqa: E402

from yam_arena.task_file import TABLE_TOP  # noqa: E402

MM = 1e-3
PULL_YAW = 0.0  # world +x
SHORT = math.radians(75.0)
el.HOUSING_HOLES = [(-68 * MM, 0.0), (0.0, 0.0), (68 * MM, 0.0)]  # M16 eyes are 63 mm wide (55 apart, they knocked each other over, thread bench 3)
el.set_hole_phases([PULL_YAW + SHORT] * 3)
el.install()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HOUSING = (0.38, -0.05, TOP + el.HOUSING_SIZE[2])  # its top face centre
TRAY = (0.38, 0.12, TOP + el.TRAY_SIZE[2] + 0.1 * MM)


def hole(i):
    x, y = el.HOUSING_HOLES[i]
    return (HOUSING[0] + x, HOUSING[1] + y)


STAND = HOUSING[2] - el.SEAT_DEPTH + 0.2 * MM  # a loose eyebolt's tip standing in a hole, dropped from 0.2 mm
OBJECTS = [el.part("housing", "housing", HOUSING, fixed=True), el.part("tray", "tray", TRAY)]
BOLTS = {"e1": (*hole(0), STAND, 0.0), "e2": (*hole(1), STAND, math.radians(50.0)),
         "e3": (*hole(2), STAND, 0.0),
         "e4": (TRAY[0], TRAY[1], TRAY[2] - el.THREAD_LEN + 0.3 * MM, math.radians(20.0))}
for k, (name, (x, y, z, yaw)) in enumerate(BOLTS.items()):
    OBJECTS += el.eyebolt_parts(name, (x, y, z), park_y=-0.3 + 0.1 * k, yaw=yaw)
NAMES = [o.name for o in OBJECTS]

SETTLE_S = 0.5
TURN_SPEED = 2 * math.pi * 1.0
TURN_MAX = 1.0  # Nm on the twin
KP_TWIN, KD_TWIN = 0.5, 0.02
LOOSE_MAX = 0.1  # Nm on the loose eyebolt
KP_LOOSE, KD_LOOSE = 0.05, 0.002
KP_UP, KD_UP = 0.3, 0.003  # Nm/rad, Nm s/rad: holding a loose eyebolt upright (inertia ~3e-5 kg m2)
START_PUSH, PUSH_DOWN = 3.0, 10.0  # N
SEATED_GAP = 0.5 * MM
ALIGN_TOL = math.radians(2.0)
HOLD_S = 1.0
LIFT = 0.002  # N over its weight


def eye_yaw(rot) -> float:
    return math.atan2(rot[1, 0], rot[0, 0])


def off_pull(rot) -> float:
    """The eye's angle from the pull direction, modulo 180 degrees (its plane either way round), in [-90, 90)."""
    return (eye_yaw(rot) - PULL_YAW + math.pi / 2) % math.pi - math.pi / 2


def run(out: Path, device: str = "cpu", max_s: float = 45.0) -> dict:
    import torch
    from inspect_robots import Scene
    from inspect_robots.types import Action

    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    emb = YamArenaEmbodiment(task=__file__, control_hz=240.0, object_jitter=0.0, cam_height=32, cam_width=32,
                             device=device)
    obs = emb.reset(Scene(id="eyebolt-thread-bench", instruction="bench"), seed=0)
    emb._env.unwrapped.cfg.sim.render_interval = 10**9
    hold = Action(data=np.asarray(obs.state["joint_pos"], dtype=float))
    base = emb._env.unwrapped
    dt = 1.0 / 240.0
    from yam_arena.scene import object_states

    def wrench(name, force, torque):
        if el._TRACK.get(name) and el._TRACK[name].engaged:
            art = base.scene.articulations[el.twin_name(name)]
            body = art.find_bodies("eyebolt")[0][0]
            comp = art.permanent_wrench_composer
        else:
            obj = base.scene.rigid_objects[name]
            body, comp = 0, obj.permanent_wrench_composer
        f = torch.tensor([[list(force)]], dtype=torch.float32, device=base.device)
        t = torch.tensor([[list(torque)]], dtype=torch.float32, device=base.device)
        comp.add_forces_and_torques_index(forces=f, torques=t, body_ids=[body], is_global=True)

    def reset_wrenches():
        for name in BOLTS:
            base.scene.rigid_objects[name].permanent_wrench_composer.reset()
            base.scene.articulations[el.twin_name(name)].permanent_wrench_composer.reset()

    phase = {"e1": "settle", "e2": "settle", "e3": "settle", "e4": "rest"}
    t_phase = dict.fromkeys(phase, 0.0)
    events = {n: {} for n in phase}
    log = {n: [] for n in phase}
    target = dict.fromkeys(phase, 0.0)
    loose = {}
    t = 0.0
    for step in range(round(max_s / dt)):
        t = step * dt
        states = object_states(emb._env, NAMES)
        reset_wrenches()
        for name in phase:
            st = states[name]
            rot = el._rot(st.quat_xyzw)
            p = phase[name]
            row = {"t": round(t, 4), "phase": p, "engaged": bool(st.engaged), "turns": round(st.turns, 4),
                   "clamp_N": round(st.clamp_force, 2), "gap_mm": None if st.gap is None else round(st.gap / MM, 3),
                   "tilt_deg": round(math.degrees(math.acos(min(1.0, max(-1.0, rot[2, 2])))), 3),
                   "eye_deg": round(math.degrees(eye_yaw(rot)), 2), "off_pull_deg": round(math.degrees(off_pull(rot)), 2),
                   "z_mm": round(st.low[2] / MM, 3), "touching": bool(st.touching)}
            log[name].append(row)
            ev = events[name]
            if st.engaged and "caught_t" not in ev:
                ev.update(caught_t=round(t, 4), snap_mm=round(el._TRACK[name].snap / MM, 3))
            if p == "settle" and t >= SETTLE_S:
                phase[name], t_phase[name] = "turn", t
                p = "turn"
            torque, force = 0.0, (0.0, 0.0, 0.0)
            axis = rot[:, 2]
            if p in ("turn", "align", "unturn"):
                sign = 1.0 if p == "unturn" else -1.0
                seated = st.engaged and st.touching and st.gap is not None and st.gap < SEATED_GAP
                if p == "turn" and seated:
                    ev.update(seated_t=round(t, 4), seated_turns=round(st.turns, 3),
                              eye_at_contact_deg=round(math.degrees(eye_yaw(rot)), 2),
                              off_pull_at_contact_deg=round(math.degrees(off_pull(rot)), 2),
                              gap_at_contact_mm=round(st.gap / MM, 3))
                    phase[name] = p = "unturn" if name == "e3" else "align"
                    t_phase[name] = t
                    sign = 1.0 if p == "unturn" else -1.0
                if p == "align" and abs(off_pull(rot)) < ALIGN_TOL:
                    ev.update(aligned_t=round(t, 4), turns_past_contact=round(st.past_contact, 3),
                              clamp_at_align_N=round(st.clamp_force, 2), gap_at_align_mm=round(st.gap / MM, 3),
                              off_pull_at_align_deg=round(math.degrees(off_pull(rot)), 2),
                              torque_at_align_Nm=round(abs(target.get(name + "_tq", 0.0)), 4))
                    phase[name], t_phase[name] = "hold", t
                    p = "hold"
                elif st.engaged:
                    angle = 2 * math.pi * -st.turns  # (clockwise turning in: negative)
                    target[name] += sign * TURN_SPEED * dt
                    target[name] = angle + sign * min(abs(target[name] - angle), TURN_MAX / KP_TWIN)
                    torque = KP_TWIN * (target[name] - angle) - KD_TWIN * st.ang_vel[2]
                    torque = max(-TURN_MAX, min(TURN_MAX, torque))
                    target[name + "_tq"] = torque
                    force = tuple(-PUSH_DOWN * axis) if p != "unturn" else (0.0, 0.0, 0.0)
                elif p == "unturn":
                    ev.setdefault("released_t", round(t, 4))
                    phase[name], t_phase[name] = "lift", t
                    ev["z_at_release_mm"] = row["z_mm"]
                    p = "lift"
                else:  # loose: follow an angle ramp about its axis, held upright, pressed lightly
                    yaw = eye_yaw(rot)
                    lt = loose.setdefault(name, {"angle": yaw, "last": yaw, "target": yaw})
                    lt["angle"] += (yaw - lt["last"] + math.pi) % (2 * math.pi) - math.pi
                    lt["last"] = yaw
                    lt["target"] += sign * TURN_SPEED * dt
                    lt["target"] = lt["angle"] + sign * min(abs(lt["target"] - lt["angle"]), LOOSE_MAX / KP_LOOSE)
                    torque = KP_LOOSE * (lt["target"] - lt["angle"]) - KD_LOOSE * st.ang_vel[2]
                    torque = max(-LOOSE_MAX, min(LOOSE_MAX, torque))
                    target[name] = 0.0
                    force = (0.0, 0.0, -START_PUSH)
                if st.engaged:
                    loose.pop(name, None)
                    wrench(name, force, tuple(torque * axis))
                elif p in ("turn", "align"):
                    w = np.asarray(st.ang_vel, dtype=float)
                    up = KP_UP * np.cross(axis, (0.0, 0.0, 1.0)) - KD_UP * (w - np.dot(w, axis) * axis)
                    wrench(name, force, tuple(torque * axis + up))
                row["torque"] = round(torque, 4)
            if p == "hold" and t >= t_phase[name] + HOLD_S:
                ev["after_hold"] = {"turns_past_contact": round(st.past_contact, 3), "clamp_N": round(st.clamp_force, 2),
                                    "gap_mm": round(st.gap / MM, 3),
                                    "off_pull_deg": round(math.degrees(off_pull(rot)), 2)}
                phase[name] = "done"
            if p == "lift":
                wrench(name, (0.0, 0.0, LIFT + el.EYEBOLT_MASS * 9.81), (0.0, 0.0, 0.0))
                if t >= t_phase[name] + 1.0:
                    phase[name] = "done"
                    ev["lifted_mm"] = round(row["z_mm"] - ev["z_at_release_mm"], 3)
        emb.step(hold)
        if all(p in ("done", "rest") for p in phase.values()) and t > 3.0:
            break

    summary = {"constants": {"pitch_mm": el.PITCH / MM, "clamp_spring_N_per_mm": el.CLAMP_SPRING / 1000,
                             "free_torque_Nm": el.FREE_TORQUE, "short_of_pull_deg": math.degrees(SHORT)}}
    for name in phase:
        rows = log[name]
        summary[name] = {"events": events[name], "final_phase": phase[name], "final": rows[-1],
                         "max_tilt_deg": round(max(r["tilt_deg"] for r in rows), 3)}
    e4 = log["e4"]
    summary["e4"]["z_range_mm"] = [min(r["z_mm"] for r in e4), max(r["z_mm"] for r in e4)]
    summary["sim_s"] = round(t, 3)
    out.mkdir(parents=True, exist_ok=True)
    (out / "el_bench_thread.json").write_text(json.dumps({"summary": summary, "log": log}, indent=1, default=float))
    return summary


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--max-s", type=float, default=45.0)
    ap.add_argument("--out", type=Path, default=Path("bench_runs"))
    args = ap.parse_args(argv)
    import ast
    for item in args.set:
        name, value = item.split("=", 1)
        if not hasattr(el, name):
            ap.error(f"--set: el_parts has no constant {name!r}")
        setattr(el, name, ast.literal_eval(value))
    summary = run(args.out, args.device, args.max_s)
    print("BENCH " + json.dumps(summary, default=float))
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
