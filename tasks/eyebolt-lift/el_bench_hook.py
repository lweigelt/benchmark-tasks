"""Hooking bench for eyebolt-lift (not a benchmark task): the planned way in (hook_plan.py, hook_path.json) followed by
a "virtual hand" (a PD force and torque on the hook) on a real eyebolt, seated in the housing and in line with the
chain, under the task's physics (240 Hz, CPU). A torque on the latch stands in for the second hand holding it open.
The arms stay in their ready pose.

    python el_bench_hook.py [--out bench_runs] [--device cpu]     # writes el_bench_hook.json, prints BENCH {...}

1. from where it hangs, the hand takes the hook to the path's start (beside the eye, tilted), the latch held open;
2. follows the path in: the hook's tip threads through the eye, the eye's top section ends in the bowl;
3. lets go of the latch: does it close over the section?
4. lets go of the hook: does it stay hooked for 3 s while the chain pulls?
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

TASK_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TASK_DIR))
import el_parts as el  # noqa: E402

from yam_arena.task_file import TABLE_TOP  # noqa: E402

MM = 1e-3
SCENE = "station"
TOP = TABLE_TOP[SCENE]
HOUSING = (0.30, -0.02, TOP + el.HOUSING_SIZE[2])
ANCHOR = (0.30, -0.16, TOP + 0.55)  # as the task
el.GANTRY_H = ANCHOR[2] - TOP
EYE_YAW = math.pi / 2  # the eye's plane along y: in line with the chain (pull toward -y)
TIP = np.array([HOUSING[0], HOUSING[1], HOUSING[2] - el.THREAD_LEN])
BAR = TIP + np.array(el.ring_top_local())  # the eye's top section's centre
el.install()

OBJECTS = [el.part("housing", "housing", HOUSING, fixed=True),
           el.part("gantry", "gantry", ANCHOR, fixed=True),
           *el.eyebolt_parts("eyebolt", tuple(TIP), park_y=-0.3, yaw=EYE_YAW, fixed=True),
           el.part("sling", "sling", ANCHOR)]
NAMES = [o.name for o in OBJECTS]

PATH = json.loads((TASK_DIR / "hook_path.json").read_text())
BOWL = np.array([el.HOOK_BOWL_C[0], 0.0, el.HOOK_BOWL_C[1]])  # in the hook's frame


def r_y(b):
    c, s = math.cos(b), math.sin(b)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def hook_pose(u_mm, w_mm, a_deg):
    """The hook's origin and rotation for a plan pose: its bowl's centre at (u, w) mm from the section's centre in the
    x-z plane (u along world x), tilted by a (counter-clockwise from x to z: about -y)."""
    rot = r_y(-math.radians(a_deg))
    bowl_world = BAR + np.array([u_mm * MM, 0.0, w_mm * MM])
    return bowl_world - rot @ BOWL, rot


KP, KD = 300.0, 6.0  # the hook weighs 50 g: explicit damping at 240 Hz needs KD * dt / m < 1 (40 blew up, runs 2-3)
KR, KDR = 0.3, 0.0015  # its inertia ~1e-5 kg m2: KR < (240 Hz)^2 * I and KDR * dt / I < 1
# (0.05 let the latch torque's reaction tilt the hook 24 degrees, run 4)
G_COMP = (el.HOOK_MASS + el.LATCH_MASS + 0.5 * el.CHAIN_N * el.CHAIN_LINK_MASS) * 9.81
LATCH_HOLD = float(os.environ.get("LATCH_HOLD", "0.02"))  # Nm opening the latch (0: not held; the eye's section must
# push it open itself. 0.03 on the 3 g latch blew up, run 2)
SPEED, TURN = 0.015, math.radians(20.0)  # m/s and rad/s along the path
LEAD_S = 3.0


def rotvec(r):
    c = max(-1.0, min(1.0, (np.trace(r) - 1) / 2))
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / (2 * math.sin(a))
    return a * v


def run(out: Path, device: str = "cpu", max_s: float = 60.0) -> dict:
    import torch
    from inspect_robots import Scene
    from inspect_robots.types import Action

    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    emb = YamArenaEmbodiment(task=__file__, control_hz=240.0, object_jitter=0.0, cam_height=32, cam_width=32,
                             device=device)
    obs = emb.reset(Scene(id="eyebolt-hook-bench", instruction="bench"), seed=0)
    emb._env.unwrapped.cfg.sim.render_interval = 10**9
    hold = Action(data=np.asarray(obs.state["joint_pos"], dtype=float))
    base = emb._env.unwrapped
    dt = 1.0 / 240.0
    from yam_arena.scene import object_states

    art = base.scene.articulations["sling"]
    hook_id, latch_id = art.find_bodies("hook")[0][0], art.find_bodies("latch")[0][0]

    def wrench(f_hook, t_hook, t_latch):
        art.permanent_wrench_composer.reset()
        f = torch.tensor([[list(f_hook), [0.0, 0.0, 0.0]]], dtype=torch.float32, device=base.device)
        t = torch.tensor([[list(t_hook), list(t_latch)]], dtype=torch.float32, device=base.device)
        art.permanent_wrench_composer.add_forces_and_torques_index(forces=f, torques=t, body_ids=[hook_id, latch_id],
                                                                  is_global=True)

    # waypoints: the path's keyframes (u, w, tilt), with a lead-in above the first one
    kf = PATH["keyframes"]
    poses = [hook_pose(*k) for k in kf]
    start = (poses[0][0] + np.array([0.04, 0.0, 0.02]), poses[0][1])
    waypoints = [start] + poses
    log, events = [], {}
    stage, wp, t_stage, from_p, from_r = "hang", 0, 0.0, None, None
    t = 0.0
    for step in range(round(max_s / dt)):
        t = step * dt
        states = object_states(emb._env, NAMES)
        st = states["sling"]
        pos, rot = np.asarray(st.pos, float), el._rot(st.quat_xyzw)
        lin, ang = np.asarray(st.lin_vel, float), np.asarray(st.ang_vel, float)
        hk = el.hooked(states["eyebolt"], st)
        p_bar = el.in_hook(BAR, st)
        row = {"t": round(t, 4), "stage": stage, "wp": wp, "hook_mm": [round(float(v) / MM, 2) for v in pos],
               "latch_deg": round(math.degrees(st.latch), 2), "bar_in_hook_mm": [round(float(v) / MM, 2) for v in p_bar],
               "hooked": hk}
        f, tq, tl = np.zeros(3), np.zeros(3), np.zeros(3)
        hold_latch = stage == "path" or (stage == "lead" and t - t_stage > LEAD_S * 0.6)  # once it's lifted
        if stage == "hang" and t >= 1.0:
            stage, t_stage, wp, from_p, from_r = "lead", t, 0, pos.copy(), rot.copy()
        if stage in ("lead", "path"):
            p_to, r_to = waypoints[wp]
            dist = float(np.linalg.norm(p_to - from_p))
            ang_d = float(np.linalg.norm(rotvec(r_to @ from_r.T)))
            dur = LEAD_S if stage == "lead" else max(0.3, dist / SPEED, ang_d / TURN)
            s = min(1.0, (t - t_stage) / dur)
            target = from_p + s * (p_to - from_p)
            r_target = r_to if s >= 1.0 else _slerp(from_r, r_to, s)
            f = KP * (target - pos) - KD * lin + np.array([0.0, 0.0, G_COMP])
            tq = KR * rotvec(r_target @ rot.T) - KDR * ang
            row["err_mm"] = round(float(np.linalg.norm(target - pos)) / MM, 2)
            row["tilt_err_deg"] = round(math.degrees(float(np.linalg.norm(rotvec(r_target @ rot.T)))), 2)
            events["max_tilt_err_deg"] = max(events.get("max_tilt_err_deg", 0.0), row["tilt_err_deg"])
            events["max_err_mm"] = max(events.get("max_err_mm", 0.0), row["err_mm"])
            if s >= 1.0 and t - t_stage >= dur + 0.2:
                events.setdefault("reached", []).append({"wp": wp, "err_mm": row["err_mm"],
                                                         "tilt_err_deg": row["tilt_err_deg"], "latch_deg": row["latch_deg"],
                                                         "bar_in_hook_mm": row["bar_in_hook_mm"]})
                wp += 1
                stage = "path"
                t_stage, from_p, from_r = t, p_to, r_to
                if wp == len(waypoints):
                    stage, t_stage = "latch", t
        elif stage == "latch":  # the hand keeps the hook at the end pose; the latch is let go
            p_to, r_to = waypoints[-1]
            f = KP * (p_to - pos) - KD * lin + np.array([0.0, 0.0, G_COMP])
            tq = KR * rotvec(r_to @ rot.T) - KDR * ang
            if t - t_stage >= 1.0:
                events["after_latch"] = {"latch_deg": row["latch_deg"], "hooked": hk, "bar_in_hook_mm": row["bar_in_hook_mm"]}
                stage, t_stage = "release", t
        elif stage == "release" and t - t_stage >= 3.0:
            events["after_release"] = {"hooked": hk, "latch_deg": row["latch_deg"], "bar_in_hook_mm": row["bar_in_hook_mm"],
                                       "hooked_frac": round(sum(r["hooked"] for r in log if r["stage"] == "release")
                                                            / max(1, sum(r["stage"] == "release" for r in log)), 3)}
            stage = "done"
        if hold_latch:
            tl = rot @ np.array([0.0, -LATCH_HOLD, 0.0])  # opening: negative about the hinge (hook's y)
            tq = tq - tl  # the hands take the reaction: an internal torque between latch and hook
        wrench(f, tq, tl)
        log.append(row)
        emb.step(hold)
        if stage == "done":
            break
    summary = {"keyframes": len(kf), "events": events, "final": log[-1], "sim_s": round(t, 3)}
    out.mkdir(parents=True, exist_ok=True)
    (out / "el_bench_hook.json").write_text(json.dumps({"summary": summary, "log": log}, indent=1, default=float))
    return summary


def _slerp(r0, r1, s):
    v = rotvec(r1 @ r0.T) * s
    a = float(np.linalg.norm(v))
    if a < 1e-9:
        return r0
    k = v / a
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return (np.eye(3) + math.sin(a) * kx + (1 - math.cos(a)) * kx @ kx) @ r0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--max-s", type=float, default=60.0)
    ap.add_argument("--out", type=Path, default=Path("bench_runs"))
    args = ap.parse_args(argv)
    summary = run(args.out, args.device, args.max_s)
    print("BENCH " + json.dumps(summary, default=float))
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
