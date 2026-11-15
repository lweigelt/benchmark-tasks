"""Scripted demo for the eyebolt-lift task, closed loop on the simulator's object poses (both arms at once).

    python demo.py --variant full --out ../../results/el_demo_N [--no-video]

Left arm, the eyebolt:
1. takes it from the tray by its eye's top section (fingers closing across the eye), carries it upright over the hole,
   centres its tip over the hole (correct, settle, measure), sets it in and turns the hand clockwise about its axis,
   keeping it upright: the thread catches;
2. screws it down in strokes of up to half a turn (grip the eye, turn the hand about the eyebolt's axis following it
   down, let go, lift a little, turn back in the air; joint 6's range planned ahead with chained IK);
3. at the shoulder, keeps tightening until the eye is in line with the chain, then lets go and moves away.
Right arm, the hook (starting once the thread has caught):
4. slides onto the hanging hook in its plane and grips its shank (centred on the measured hook, retried if missed),
   lifts it and holds it clear of the left hand;
5. once the eyebolt is in line, carries the hook beside the eye and follows the planned way in (hook_plan.py,
   hook_path.json: 3 mm clearance, the hook kept at its ~20 degree tilt in the grip), correcting the hand by the hook's
   measured position: the tip passes through the eye, the section pushes the latch open and drops into the bowl,
   the latch closes;
6. lets go and moves away.

Variants (one per score level): full 1.0; hold (keeps holding the hook) 0.75; no_hook (right arm idle) 0.5; no_align
(stops at the shoulder, right arm idle) 0.25.

Writes ``result.json``, ``steps.jsonl`` and (unless ``--no-video``) one H.264 video per camera to ``--out``.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
INSTRUCTION = ("Fit the lifting eyebolt into the part before a lift: screw it down to its shoulder, tighten it until "
               "its eye is in line with the chain, then hook the chain's hook into the eye and let go.")
VARIANTS = ("full", "hold", "no_hook", "no_align")  # 1.0; hooked but not let go 0.75; in line, not hooked 0.5;
# seated, not turned in line (and not hooked) 0.25
CAMERAS = ("top_cam", "left_cam", "right_cam")
CONTROL_HZ = 20.0
DT = 1.0 / CONTROL_HZ
MAX_STEPS = 7000  # the task's run limit
MM = 1e-3

PAD_TIP = 0.013  # pad tips beyond the TCP
OPEN_RING = 0.30  # ~28 mm: open round the eye's section (M16: 14 mm)
GRIP_DEPTH = 18 * MM  # the pad tips this far below the eye's top (low: the hand stays under ~14 cm above the table,
# as high as the arms reach pointing down; gripping 12 mm deep and carrying 30 mm above the hole was out of reach, run 1)
ABOVE = 0.03
ABOVE_HOLE = 0.012
JUST_ABOVE = 1.0 * MM
CENTRE_TOL = 0.2 * MM
CENTRE_SETTLE = 0.5
UPRIGHT_TOL = math.radians(2.0)
SEAT_TRIES = 3
START_TURN = math.radians(30.0)
START_TRIES, START_TILT_GAIN = 3, 0.15
START_SERVO, START_SERVO_MAX = 0.2, 1.5 * MM
START_PRESS = 0.5 * MM
SETTLE_TOL, SETTLE_MAX_S = 0.5 * MM, 2.0
STROKE = math.pi  # each stroke turns at most half a turn ...
MIN_STROKE = math.radians(25.0)  # ... and at least this (the eye repeats every half turn but joint 6 has only 240
# degrees: depending on the eye's angle, the best grip leaves 60-180 degrees of turn, run 3)
PLAN_STEP = math.radians(15.0)
WRIST_Q_LIMIT = math.radians(115.0)  # joint 6's range is +-120 degrees
TURN_RATE = math.radians(90.0)
LIFT_OFF = 15 * MM  # between strokes the open hand lifts this far before turning back
MAX_STROKES = 40
ALIGN_TOL_DEG = 3.0  # the demo aims the eye within this of the chain (the scorer allows 15)
OPEN_HOOK = 0.25  # ~24 mm: round the hook's 7 mm section
HOOK_PRE = 0.06  # the hand starts this far beside the hook, in its plane
HOOK_PARK = (0.28, -0.24, 0.0)  # where the right hand holds the hook (its origin; z set below) while the left screws
GRIP_IN_LINE = False  # grip the hook in line with its hanging tilt (slipped, run 28) or pointing straight down
HOOK_FLIP = {"1": True, "0": False}.get(os.environ.get("EL_HOOK_FLIP", ""))  # force the mirrored way in, or None:
# by the hook's yaw
GRIP_TRIES, GRIP_TRACK_S, GRIP_CENTRE_TOL = 3, 1.0, 3 * MM  # take the hook: follow it until centred, then close
HOOK_TCP_Z = -5 * MM  # the hand's TCP on the hook's shank, in the hook's frame (pad tips 18 mm down)
HOOK_WAIT = (0.0, -0.10, 0.0)  # waiting beside the eye (from the path's start), clear of the screwing hand (7 cm away,
# the two hands met while the left placed the eyebolt and tipped it 40 degrees, run 7)
HOOK_LEAD = (0.0, 0.0, 0.0)  # (4 cm further out was beyond reach that low, run 22)  # the way in starts here from the path's first pose
HOOK_SPEED, HOOK_TURN = 0.010, math.radians(15.0)  # m/s, rad/s along the planned way in
HOOK_SERVO, HOOK_SERVO_MAX = 0.15, 6 * MM
HOOK_TRACK = float(os.environ.get("EL_HOOK_TRACK", "0"))  # per step: how fast the hand's offset to the hook follows
# its measured value along the way in (0: fixed at the grip, with the position servo instead). The eye's section
# pushing the latch open twisted the hook 15-30 degrees in the grip, and the fixed offset led the hand astray (runs 33-35)
AFTER_DONE_S = 3.0
ABORT_EARLY = True
IK_ABORT_MM, IK_ABORT_S = 20.0, 2.0


def smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Arm:
    """One arm's commanded TCP pose and gripper, turned into joint targets by IK (warm-started)."""

    def __init__(self, side: str, base, q0, ik, grip_rotation):
        self.side, self.base, self.ik = side, np.asarray(base, dtype=float), ik
        self.q = np.array(q0, dtype=float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.grip = 1.0
        self.down = grip_rotation(0.0)  # pointing down, fingers closing along world y
        self.ik_err = (0.0, 0.0)

    def action(self) -> np.ndarray:
        q, pe, re = self.ik.solve(self.pos, self.ik.matrix_to_quat_xyzw(self.rot), self.q, base_pos=self.base, iters=60)
        self.q, self.ik_err = q, (pe, re)
        return np.r_[q, self.grip]


def make_embodiment(video: bool, device: str = "cpu"):
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    size = (360, 640) if video else (64, 64)
    return YamArenaEmbodiment(task=str(TASK), control_hz=CONTROL_HZ, object_jitter=0.0, cam_height=size[0],
                              cam_width=size[1], device=device)


def set_rendering(emb, on: bool) -> None:
    emb._env.unwrapped.cfg.sim.render_interval = emb._env.unwrapped.cfg.decimation if on else 10**9


def run(variant: str, out: Path, video: bool, max_steps: int, emb=None) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack, split

    from yam_arena import ik, station
    from yam_arena.demo import grip_rotation

    emb = emb or make_embodiment(video)
    obs = emb.reset(Scene(id=f"eyebolt-{variant}", instruction=INSTRUCTION), seed=0)
    set_rendering(emb, video)
    task = emb.task.module
    el = task.el
    from yam_arena.scene import object_states

    left_q, right_q = split(np.asarray(obs.state["joint_pos"]))
    arms = {"left": Arm("left", station.BASES["left"], left_q[:6], ik, grip_rotation),
            "right": Arm("right", station.BASES["right"], right_q[:6], ik, grip_rotation)}
    world = {"states": object_states(emb._env, emb.task.names), "obs": obs, "step": 0}
    report: dict = {"bolt": {}}
    flags: dict = {}
    phase = {"left": "", "right": ""}
    hole = np.array(task.HOLE)

    # -- measurements -------------------------------------------------------------------------------------------
    def bolt():
        """(tip, rotation) of the eyebolt (whichever body is live)."""
        return task.bolt_tip(world["states"]["eyebolt"])

    def ring_top():
        tip, r = bolt()
        return tip + r @ np.array(el.ring_top_local()), r

    def eye_yaw() -> float:
        _, r = bolt()
        return math.atan2(r[1, 0], r[0, 0])

    def tcp_measured(arm: Arm):
        left, right = split(np.asarray(world["obs"].state["joint_pos"]))
        return ik.tcp_pose((left if arm.side == "left" else right)[:6], arm.base)[0]

    # -- motions --------------------------------------------------------------------------------------------------
    def move(arm: Arm, pos, duration: float, grip=None, rot=None, label=""):
        phase[arm.side] = label
        p0, r0, g0 = arm.pos.copy(), arm.rot.copy(), arm.grip
        p1 = np.asarray(pos, dtype=float)
        r1 = r0 if rot is None else np.asarray(rot, dtype=float)
        g1 = g0 if grip is None else grip
        n = max(1, round(duration / DT))
        for i in range(1, n + 1):
            s = smooth(i / n)
            arm.pos = p0 + s * (p1 - p0)
            arm.rot = ik.slerp_matrix(r0, r1, s)
            arm.grip = g0 + s * (g1 - g0)
            yield

    def wait(arm: Arm, duration: float, label=""):
        phase[arm.side] = label
        for _ in range(max(1, round(duration / DT))):
            yield

    def solve_best(arm: Arm, pos, rot):
        """IK from several starting joints (the current ones, zeros, random): the most accurate solution."""
        best = None
        for k in range(10):
            seed = arm.q if k == 0 else (np.zeros(6) if k == 1 else np.random.default_rng(k).uniform(-1.5, 1.5, 6))
            q, pe, re = ik.solve(np.asarray(pos), ik.matrix_to_quat_xyzw(rot), seed, base_pos=arm.base, iters=200)
            if best is None or pe + 0.01 * re < best[1]:
                best = (q, pe + 0.01 * re)
        return best[0]

    def move_joints(arm: Arm, q_to, duration: float, label=""):
        """Move by interpolating the joints (not the hand's pose): from one IK branch to another without a jump."""
        phase[arm.side] = label
        q0 = arm.q.copy()
        n = max(1, round(duration / DT))
        for i in range(1, n + 1):
            q = q0 + smooth(i / n) * (np.asarray(q_to) - q0)
            arm.q = q
            arm.pos, arm.rot = ik.tcp_pose(q, arm.base)
            yield

    def wait_for(arm: Arm, flag: str, label=""):
        phase[arm.side] = label
        while not flags.get(flag):
            yield

    def settle_at(arm: Arm, label="", tol=SETTLE_TOL, max_s=SETTLE_MAX_S):
        phase[arm.side] = label
        for _ in range(round(max_s / DT)):
            if np.linalg.norm(tcp_measured(arm) - arm.pos) < tol:
                break
            yield

    def hand_for(r_target, p_target, offset):
        r_off, p_off = offset
        r_hand = r_target @ r_off.T
        return r_hand, p_target - r_hand @ p_off

    def offset_of(arm: Arm, pose):
        p_part, r_part = pose
        return arm.rot.T @ r_part, arm.rot.T @ (p_part - arm.pos)

    def ik_ok(arm: Arm, pos, rot) -> bool:
        _, pe, re = ik.solve(np.asarray(pos), ik.matrix_to_quat_xyzw(rot), arm.q, base_pos=arm.base, iters=80)
        return pe < 2 * MM and re < 0.02

    def grasp_pose(arm: Arm, hand_yaw: float):
        """TCP pose taking the eye's top section, fingers closing across the eye (hand_yaw: the eye's in-plane
        direction, or that plus a half turn)."""
        top, _ = ring_top()
        rot = rot_z(hand_yaw) @ arm.down
        pos = np.array([top[0], top[1], top[2] + el.RING_RS - GRIP_DEPTH + PAD_TIP])
        return pos, rot

    # -- left arm: the eyebolt --------------------------------------------------------------------------------------
    def pick():
        arm = arms["left"]
        pos, rot = grasp_pose(arm, eye_yaw())
        yield from move(arm, pos + (0, 0, ABOVE), 2.0, grip=OPEN_RING, rot=rot, label="over the eyebolt")
        yield from move(arm, pos, 1.2, label="down round the eye")
        yield from settle_at(arm, "settle round the eye")
        yield from move(arm, pos, 0.6, grip=0.0, label="grip the eye")
        yield from move(arm, pos + (0, 0, 0.05), 1.2, label="lift it out of the tray")
        yield from wait(arm, 0.3, "settle")
        tip, _ = bolt()
        report["bolt"]["picked"] = bool(tip[2] > task.TRAY_ORIGIN[2] + 5 * MM)  # its thread's tip out of the tray

    def place():
        arm = arms["left"]
        tip, r = bolt()
        off = offset_of(arm, (tip, r))
        # carry it upright, at the turn of the hand the arm reaches best above the hole
        yaw0 = math.atan2(r[1, 0], r[0, 0])
        best = None
        for k in range(-6, 6):
            cand = rot_z(yaw0 + k * math.pi / 6)
            r_c, p_c = hand_for(cand, hole + (0, 0, ABOVE_HOLE), off)
            _, pe, re = ik.solve(p_c, ik.matrix_to_quat_xyzw(r_c), arm.q, base_pos=arm.base, iters=80)
            err = pe + 0.01 * re + 1e-4 * abs(k)
            if best is None or err < best[0]:
                best = (err, cand)
        r_h, p_h = hand_for(best[1], hole + (0, 0, ABOVE_HOLE), off)
        yield from move(arm, (p_h[0], p_h[1], max(p_h[2], arm.pos[2])), 2.0, rot=r_h, label="carry")
        yield from move(arm, p_h, 0.8, label="over the hole")

        def centre(tries, tag, upright=True):
            err = np.zeros(2)
            for i in range(tries + 1):
                yield from wait(arm, CENTRE_SETTLE, tag)
                tip, r = bolt()
                err = (hole - tip)[:2]
                axis = r[:, 2]
                tilt = math.acos(min(1.0, axis[2]))
                if i == tries or (np.linalg.norm(err) < CENTRE_TOL and (tilt < UPRIGHT_TOL or not upright)):
                    break
                if not upright:
                    tilt = 0.0
                k = np.cross(axis, (0.0, 0.0, 1.0))
                kn = np.linalg.norm(k)
                r_c = np.eye(3)
                if kn > 1e-9:
                    k = k / kn
                    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
                    r_c = np.eye(3) + math.sin(tilt) * kx + (1 - math.cos(tilt)) * kx @ kx
                new_pos = tip + r_c @ (arm.pos - tip) + (err[0], err[1], 0.0)
                yield from move(arm, new_pos, 0.4, rot=r_c @ arm.rot, label=tag)
            return float(np.linalg.norm(err))

        e1 = yield from centre(4, "centre over the hole")
        e2 = 0.0
        for attempt in range(SEAT_TRIES):
            tip, _ = bolt()
            yield from move(arm, arm.pos - (0, 0, tip[2] - hole[2] - JUST_ABOVE), 1.0, label="just above the hole")
            e2 = yield from centre(4, "centre just above the hole", upright=False)
            tip, _ = bolt()
            yield from move(arm, arm.pos - (0, 0, tip[2] - hole[2] + el.SEAT_DEPTH + START_PRESS), 0.8,
                            label="set it in")
            yield from wait(arm, 0.4, "settle in the hole")
            tip, _ = bolt()
            if tip[2] < hole[2] - el.ENGAGE_MIN_DEPTH:
                break
            yield from move(arm, arm.pos + (0, 0, JUST_ABOVE + 1.5 * MM), 0.6, label="up, not in the hole")
        tip, r = bolt()
        report["bolt"].update(centre_err_mm=[round(e1 / MM, 3), round(e2 / MM, 3)], seat_tries=attempt + 1,
                              seated_mm=[round(float(v) / MM, 2) for v in tip - hole],
                              tilt_deg=round(math.degrees(math.acos(min(1.0, r[2, 2]))), 2))
        # start the thread: turn the hand clockwise about the eyebolt's axis, keeping it upright
        p0, r0 = arm.pos.copy(), arm.rot.copy()
        about = np.array([tip[0], tip[1], p0[2]])
        n = round(2.0 / DT)
        for start_try in range(START_TRIES):
            if start_try:
                yield from move(arm, p0, 1.0, rot=r0, label="turn back, try again")
            servo, r_tilt = np.zeros(3), np.eye(3)
            for i in range(1, n + 1):
                rz = rot_z(-START_TURN * smooth(i / n))
                tip, r_s = bolt()
                servo[:2] = np.clip(servo[:2] + START_SERVO * (hole - tip)[:2], -START_SERVO_MAX, START_SERVO_MAX)
                k = np.cross(r_s[:, 2], (0.0, 0.0, 1.0))
                kn = float(np.linalg.norm(k))
                if kn > 1e-6:
                    k = k / kn
                    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
                    ang = START_TILT_GAIN * math.asin(min(1.0, kn))
                    r_tilt = (np.eye(3) + math.sin(ang) * kx + (1 - math.cos(ang)) * kx @ kx) @ r_tilt
                pos = about + rz @ (p0 - about) + servo
                arm.rot, arm.pos = r_tilt @ rz @ r0, tip + r_tilt @ (pos - tip)
                phase["left"] = "start the thread"
                yield
                if world["states"]["eyebolt"].engaged:
                    break
            if world["states"]["eyebolt"].engaged:
                break
        report["bolt"].update(start_tries=start_try + 1, caught=bool(world["states"]["eyebolt"].engaged))
        yield from move(arm, arm.pos, 0.5, grip=OPEN_RING, label="let go")
        yield from move(arm, arm.pos + (0, 0, LIFT_OFF), 0.6, label="up")

    def hand_yaw(arm: Arm, rot=None) -> float:
        r = (arm.rot if rot is None else rot) @ arm.down.T
        return math.atan2(r[1, 0], r[0, 0])

    def stroke_plan(arm: Arm):
        """(grip yaw, how far the wrist can turn clockwise from it, joint 6 there): over the grip yaws (the eye's
        direction plus whole half turns) and several starting values of joint 6, the longest turn, checked by stepping
        it with chained IK inside joint 6's limits (checked at the two ends only, a stroke ran into joint 6's +-120
        degrees half way, run 2; solved from the current joints only, every grip came out near the limit, run 4)."""
        ey = eye_yaw()
        best = None
        for k in range(-1, 2):
            yaw = ey + k * math.pi
            pos, rot = grasp_pose(arm, yaw)
            for q6 in [None] + [math.radians(v) for v in range(-110, 111, 20)]:
                seed = arm.q.copy()
                if q6 is not None:
                    seed[5] = q6
                q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), seed, base_pos=arm.base, iters=80)
                if pe > 1.5 * MM or re > 0.02 or abs(q[5]) > WRIST_Q_LIMIT:
                    continue
                q_start, reach = q.copy(), 0.0
                for n in range(1, round(STROKE / PLAN_STEP) + 1):
                    a = n * PLAN_STEP
                    q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot_z(-a) @ rot), q, base_pos=arm.base,
                                         iters=60)
                    if pe > 1.5 * MM or re > 0.02 or abs(q[5]) > WRIST_Q_LIMIT:
                        break
                    reach = a
                if best is None or reach > best[1] + 1e-6:
                    best = (yaw, reach, float(q_start[5]))
        return best

    def turn_to(arm: Arm, pos, yaw: float, q6: float, duration: float, label=""):
        """Move to ``pos`` while turning the hand to ``yaw`` the way that takes joint 6 to ``q6`` (not the shortest
        rotation: half a turn back is ambiguous)."""
        phase[arm.side] = label
        y0 = hand_yaw(arm)
        # which way does the hand's yaw go when joint 6 increases?
        q_t = arm.q.copy()
        q_t[5] += 0.1
        _, r_t = ik.tcp_pose(q_t, arm.base)
        sgn = 1.0 if wrap(hand_yaw(arm, r_t) - y0) > 0 else -1.0
        want = sgn * (q6 - arm.q[5])
        d = wrap(yaw - y0)
        d += 2 * math.pi * round((want - d) / (2 * math.pi))
        p0, r0 = arm.pos.copy(), arm.rot.copy()
        p1 = np.asarray(pos, dtype=float)
        n = max(1, round(duration / DT))
        for i in range(1, n + 1):
            s_ = smooth(i / n)
            arm.pos = p0 + s_ * (p1 - p0)
            arm.rot = rot_z(d * s_) @ r0
            yield

    def screw_down():
        arm = arms["left"]
        rec = report["bolt"]
        rec["strokes"] = 0
        while rec["strokes"] < MAX_STROKES:
            st = world["states"]["eyebolt"]
            seated = task.seated(world["states"])
            off = task.eye_off_pull_deg(world["states"])
            if seated and abs(off) <= ALIGN_TOL_DEG:
                rec["aligned_step"] = world["step"]
                break
            if seated and variant == "no_align":  # stops at the shoulder, the eye 75 degrees off the chain
                break
            plan = stroke_plan(arm)
            if plan is None or plan[1] < MIN_STROKE:
                rec["last_plan"] = None if plan is None else [round(math.degrees(v), 1) for v in plan]
                rec["gave_up"] = "no wrist yaw reaches a stroke"
                return
            yaw, reach, q6 = plan
            rec.setdefault("plans_deg", []).append(round(math.degrees(reach)))
            rec["strokes"] += 1
            pos, rot = grasp_pose(arm, yaw)
            arm.grip = OPEN_RING
            yield from turn_to(arm, pos + (0, 0, LIFT_OFF), yaw, q6, 1.5, label=f"stroke {rec['strokes']}: turn back")
            yield from move(arm, pos, 0.6, label=f"stroke {rec['strokes']}: down round the eye")
            yield from settle_at(arm, f"stroke {rec['strokes']}: settle", max_s=1.0)
            yield from move(arm, arm.pos, 0.4, grip=0.0, label=f"stroke {rec['strokes']}: grip")
            # how far this stroke turns: a half turn, or (seated) just what brings the eye in line (clockwise)
            turn = reach
            if task.seated(world["states"]):
                o = math.radians(task.eye_off_pull_deg(world["states"]))
                turn = min(reach, o if o > 0 else o + math.pi)
            tip, _ = bolt()
            p0, r0 = arm.pos.copy(), arm.rot.copy()
            about = np.array([tip[0], tip[1], p0[2]])
            n = max(1, round(turn / TURN_RATE / DT))
            last = eye_yaw()
            followed = 0.0
            for i in range(1, n + 1):
                a = -turn * smooth(i / n)
                rz = rot_z(a)
                y = eye_yaw()
                followed += wrap(y - last)
                last = y
                # follow the eyebolt down as it turns in (1.5 mm per turn), by its measured turning
                dz = -followed / (2 * math.pi) * el.PITCH if not task.seated(world["states"]) else 0.0
                arm.rot, arm.pos = rz @ r0, about + rz @ (p0 - about) - (0, 0, dz)
                phase["left"] = f"stroke {rec['strokes']}: turn"
                yield
                if task.seated(world["states"]) and "seated_step" not in rec:
                    rec["seated_step"] = world["step"]
                    rec["eye_at_seat_deg"] = round(task.eye_off_pull_deg(world["states"]), 2)
                    break  # re-grip and turn just as far as the alignment needs
            yield from move(arm, arm.pos, 0.4, grip=OPEN_RING, label=f"stroke {rec['strokes']}: let go")
            yield from move(arm, arm.pos + (0, 0, LIFT_OFF), 0.5, label=f"stroke {rec['strokes']}: up")
        st = world["states"]["eyebolt"]
        rec.update(final_off_pull_deg=round(task.eye_off_pull_deg(world["states"]), 2),
                   gap_mm=None if st.gap is None else round(st.gap / MM, 3), clamp_N=round(st.clamp_force, 1),
                   turns=round(st.turns, 3))

    def left_script():
        arm = arms["left"]
        yield from wait(arm, 0.5, "start")
        yield from pick()
        if not report["bolt"].get("picked"):
            report["abort"] = "eyebolt not picked up"
            while True:
                yield
        yield from place()
        flags["bolt_caught"] = bool(report["bolt"].get("caught"))
        if not report["bolt"].get("caught"):
            report["abort"] = "thread didn't catch"
            while True:
                yield
        yield from screw_down()
        yield from move(arm, arm.pos + (-0.03, 0.04, 0.02), 1.0, grip=OPEN_RING, label="away")
        yield from move(arm, arm.pos + (-0.05, 0.08, 0.0), 1.5, label="away")
        flags["bolt_done"] = True
        phase["left"] = "done"
        while True:
            yield

    # -- right arm: the hook ------------------------------------------------------------------------------------------
    path = json.loads((HERE / "hook_path.json").read_text())["keyframes"]
    bowl_local = np.array([el.HOOK_BOWL_C[0], 0.0, el.HOOK_BOWL_C[1]])

    def hook():
        st = world["states"]["sling"]
        return np.asarray(st.pos, dtype=float), el._rot(st.quat_xyzw)

    def r_y(b):
        c, s_ = math.cos(b), math.sin(b)
        return np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])

    def hook_target(u_mm, w_mm, a_deg, bar, flip=False):
        """The hook's origin and rotation for a plan pose: its bowl's centre at (u, w) mm from the eye's top section in
        the plane across the eye (u along world x), tilted by a about -y (hook_plan.py's frame). ``flip``: the same
        mirrored across the eye's plane (the hook turned half a turn, coming from the other side; the eye is
        symmetric), for a hook gripped the other way round."""
        rot = r_y(-math.radians(a_deg))
        off = np.array([u_mm * MM, 0.0, w_mm * MM])
        if flip:
            rot = rot_z(math.pi) @ rot
            off[0] = -off[0]
        return bar + off - rot @ bowl_local, rot

    def right_script():
        arm = arms["right"]
        home_pos, home_rot = arm.pos.copy(), arm.rot.copy()
        if variant in ("no_hook", "no_align"):  # the right arm stays where it is
            yield from wait_for(arm, "bolt_done", "idle")
            report["script_done_step"] = world["step"]
            while True:
                phase["right"] = "idle"
                yield
        # fetch the hook once the eyebolt has caught its thread (the left hand's placing and starting are the delicate
        # part: with the right hand busy at the hook nearby, the eyebolt tipped or didn't catch, runs 7-10)
        yield from wait_for(arm, "bolt_caught", "wait until the eyebolt has caught")
        # take the hanging hook by its top: fingers across its plane, pad tips 18 mm down its shank
        # the chain hangs straight up from the hook: come in beside the hook, in its plane, at its height, and slide
        # onto it so the chain passes between the fingers (coming across the plane from the start pose, a finger swept
        # into the chain and swung the hook 8 cm, runs 7-9)
        report["hook"] = {}

        def grip_target():
            """The hand in line with the hanging hook (which hangs ~14 degrees tilted, its weight under the bowl):
            gripped pointing straight down, the hook sat tilted in the hand and the path's own tilt added to it, out
            of reach (run 27)."""
            o_, r_ = hook()
            if not GRIP_IN_LINE:
                yaw_ = math.atan2(r_[1, 0], r_[0, 0])
                return o_ + r_ @ np.array([0.0, 0.0, HOOK_TCP_Z]), rot_z(yaw_) @ arm.down, r_
            return o_ + r_ @ np.array([0.0, 0.0, HOOK_TCP_Z]), r_ @ arm.down, r_

        held = False
        for attempt in range(GRIP_TRIES):
            tcp, grip_rot, r = grip_target()
            side = r[:, 0] * (1.0 if r[0, 0] <= 0 else -1.0)  # in the hook's plane, toward the arms (from the far
            # side the right arm didn't reach that low, run 14)
            pre = tcp + HOOK_PRE * side
            yield from move(arm, pre + (0, 0, 0.03), 2.5 if attempt == 0 else 1.0, grip=OPEN_HOOK, rot=grip_rot,
                            label=f"beside the hook ({attempt + 1})")
            yield from move(arm, pre, 1.0, label="down beside the hook")
            # slide on, following the hook as it moves (the hand's base pushes the chain above it and swings it)
            n = round(1.5 / DT)
            for i in range(1, n + 1):
                tcp, grip_rot, _ = grip_target()
                s_ = smooth(i / n)
                arm.pos, arm.rot = pre + s_ * (tcp - pre), grip_rot
                phase["right"] = "slide onto the hook"
                yield
            centred = False
            for _ in range(round(GRIP_TRACK_S / DT)):
                tcp, grip_rot, _ = grip_target()
                arm.pos, arm.rot = tcp, grip_rot
                phase["right"] = "follow the hook"
                yield
                close_dir = arm.rot @ arm.down.T @ np.array([0.0, 1.0, 0.0])
                err = tcp - tcp_measured(arm)
                if abs(float(np.dot(err, close_dir))) < GRIP_CENTRE_TOL and np.linalg.norm(err) < 3 * GRIP_CENTRE_TOL:
                    centred = True
                    break
            yield from move(arm, arm.pos, 0.5, grip=0.0, label="grip the hook")
            o_before = hook()[0].copy()
            yield from move(arm, arm.pos + (0, 0, 0.015), 0.6, label="lift the hook (check)")
            held = hook()[0][2] - o_before[2] > 0.010
            report["hook"].setdefault("tries", []).append({"centred": centred, "held": bool(held)})
            if held:
                break
            yield from move(arm, arm.pos, 0.4, grip=OPEN_HOOK, label="missed: open")
            yield from move(arm, arm.pos + 0.03 * side + (0, 0, 0.02), 0.8, label="missed: back off")
        if not held:
            report["abort"] = "right hand couldn't take the hook"
            while True:
                yield
        report["hook"]["gripped_step"] = world["step"]
        off = offset_of(arm, hook())
        # wait beside the eye, clear of the left hand, until the eyebolt is seated and in line
        bar0 = np.array(task.HOLE) + np.array([0.0, 0.0, (el.RING_Z + el.RING_RC - el.THREAD_LEN)])
        # which way round: the plan or its mirror image, whichever is nearer the hook's yaw as gripped (turning the
        # hand half a turn on the way there is beyond the wrist, run 23)
        _, r_g = hook()
        flip = abs(wrap(math.atan2(r_g[1, 0], r_g[0, 0]))) > math.pi / 2
        if HOOK_FLIP is not None:
            flip = HOOK_FLIP
        report["hook"]["flip"] = bool(flip)
        p_lead, r_lead = hook_target(*path[0], bar0, flip)
        p_wait = p_lead + np.array(HOOK_WAIT)
        # wait well clear of the screwing hand until the eyebolt is done (holding the hook 10 cm from the eye, the
        # left hand's strokes ran into the right hand and stalled at 8.3 turns, run 13)
        o, r = hook()
        park = np.array([HOOK_PARK[0], HOOK_PARK[1], task.TOP + 0.10])
        r_h, p_h = hand_for(r, park, off)
        yield from move(arm, p_h, 2.0, rot=r_h, label="hold the hook clear")
        yield from wait_for(arm, "bolt_done", "hold the hook, wait for the eyebolt")
        r_h, p_h = hand_for(r_lead, p_wait, off)
        # in joint space: from where the arm waited, the hand's straight way there ran the IK into an elbow
        # configuration that couldn't follow the path (run 25)
        q_w = solve_best(arm, p_h, r_h)
        p_chk, r_chk = ik.tcp_pose(q_w, arm.base)
        report["hook"]["carry_target"] = {"p_wait_hook": [round(float(v), 4) for v in p_wait],
                                          "p_hand": [round(float(v), 4) for v in p_h],
                                          "ik_err_mm": round(float(np.linalg.norm(p_chk - p_h)) / MM, 2),
                                          "hand_tilt_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, -r_h[2, 2])))), 1),
                                          "off_p_mm": [round(float(v) / MM, 1) for v in off[1]]}
        yield from move_joints(arm, q_w, 3.0, label="carry the hook beside the eye")
        if not task.aligned(world["states"]):
            report["hook"]["skipped"] = "eyebolt not seated and in line"
            while True:
                phase["right"] = "wait"
                yield
        top, _ = ring_top()
        # follow the planned way in, correcting the hand by the hook's measured offset (the wrist is soft)
        targets = [hook_target(*k, top, flip) for k in path]
        corr = np.zeros(3)
        cur_p, cur_r = hook()
        for k, (p_t, r_t) in enumerate(targets):
            dist = float(np.linalg.norm(p_t - cur_p))
            ang = math.acos(max(-1.0, min(1.0, (np.trace(r_t @ cur_r.T) - 1) / 2)))
            n = max(2, round(max(dist / HOOK_SPEED, ang / HOOK_TURN) / DT))
            p0, r0 = cur_p, cur_r
            for i in range(1, n + 1):
                s_ = i / n
                p_i = p0 + s_ * (p_t - p0)
                r_i = ik.slerp_matrix(r0, r_t, s_)
                o, r_o = hook()
                if HOOK_TRACK > 0:  # the hook's pose in the (commanded) hand, as it is now
                    r_now, p_now = offset_of(arm, (o, r_o))
                    off = (ik.slerp_matrix(off[0], r_now, HOOK_TRACK), off[1] + HOOK_TRACK * (p_now - off[1]))
                else:
                    corr = np.clip(corr + HOOK_SERVO * (p_i - o), -HOOK_SERVO_MAX, HOOK_SERVO_MAX)
                r_h, p_h = hand_for(r_i, p_i, off)
                arm.rot, arm.pos = r_h, p_h + corr
                phase["right"] = f"hook in: {k}/{len(targets) - 1}"
                yield
            cur_p, cur_r = p_t, r_t
            o, _ = hook()
            report["hook"].setdefault("err_mm", []).append(round(float(np.linalg.norm(p_t - o)) / MM, 1))
        yield from wait(arm, 0.5, "hold the hook")
        report["hook"]["hooked_before_release"] = bool(task.hooked(world["states"]))
        if variant == "hold":  # keeps holding the hook
            yield from wait(arm, 3.0, "hold the hook")
            report["script_done_step"] = world["step"]
            while True:
                phase["right"] = "hold the hook"
                yield
        yield from move(arm, arm.pos, 0.6, grip=OPEN_HOOK, label="let go of the hook")
        yield from move(arm, arm.pos + (0.0, -0.03, 0.02), 0.8, label="up off the hook")
        yield from move(arm, home_pos, 2.5, rot=home_rot, label="away")
        report["hook"]["hooked_after_release"] = bool(task.hooked(world["states"]))
        report["script_done_step"] = world["step"]
        phase["right"] = "done"
        while True:
            yield

    def tip_positions():
        art = emb._env.unwrapped.scene.articulations.get("right_robot")
        if art is None:
            return None
        return {b: [round(float(v), 4) for v in el._body_pose(emb._env.unwrapped, "right_robot", b)[0]]
                for b in ("tip_left", "tip_right", "gripper")}

    scripts = {"left": left_script(), "right": right_script()}
    frames = {c: [] for c in CAMERAS}
    log = []
    result = None
    t0 = time.time()
    for step in range(1, max_steps + 1):
        world["step"] = step
        try:
            for side in ("left", "right"):
                next(scripts[side])
        except Exception as exc:
            import traceback

            report["crash"] = f"step {step}: {type(exc).__name__}: {exc}"
            report["crash_trace"] = traceback.format_exc()[-1500:]
            break
        res = emb.step(Action(data=pack(arms["left"].action(), arms["right"].action())))
        world["obs"] = res.observation
        world["states"] = object_states(emb._env, emb.task.names)
        st = world["states"]
        b = st["eyebolt"]
        tip, r = bolt()
        row = {"step": step, "left": phase["left"], "right": phase["right"], "score": res.info.get("score"),
               "engaged": bool(b.engaged), "turns": round(float(b.turns), 3),
               "gap_mm": None if b.gap is None else round(b.gap / MM, 3), "clamp_N": round(float(b.clamp_force), 1),
               "touching": bool(b.touching), "eye_off_pull_deg": round(task.eye_off_pull_deg(st), 2),
               "tilt_deg": round(math.degrees(math.acos(min(1.0, r[2, 2]))), 2),
               "tip": [round(float(v), 4) for v in tip],
               "l_ik_mm": round(arms["left"].ik_err[0] / MM, 2),
               "l_cmd": [round(float(v), 4) for v in arms["left"].pos],
               "l_meas": [round(float(v), 4) for v in tcp_measured(arms["left"])],
               "hook_hand_mm": round(float(st["sling"].hand_dist) / MM, 1),
               "r_ik_mm": round(arms["right"].ik_err[0] / MM, 2),
               "r_meas": [round(float(v), 4) for v in tcp_measured(arms["right"])],
               "r_tips": tip_positions(),
               "hook": [round(float(v), 4) for v in st["sling"].pos], "latch_deg": round(math.degrees(st["sling"].latch), 1),
               "hook_up": [round(float(v), 3) for v in el._rot(st["sling"].quat_xyzw)[:, 2]],
               "hook_in_hand_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, float(
                   (el._rot(st["sling"].quat_xyzw)[:, 2]) @ -(ik.tcp_pose(split(np.asarray(world["obs"].state["joint_pos"]))[1][:6],
                                                                          arms["right"].base)[1][:, 2])))))), 1),
               "hooked": bool(task.hooked(st))}
        log.append(row)
        if video:
            for c in CAMERAS:
                frames[c].append(res.observation.images[c])
        if "abort" not in report:
            worst = max(row["l_ik_mm"], row["r_ik_mm"])
            report["_ik_bad"] = report.get("_ik_bad", 0) + 1 if worst > IK_ABORT_MM else 0
            if report["_ik_bad"] >= round(IK_ABORT_S / DT):
                report["abort"] = f"step {step}: an arm can't reach its target ({worst:.0f} mm)"
        result = res
        if "abort" in report and ABORT_EARLY:
            break
        if res.terminated and "success_step" not in report:
            report["success_step"] = step
        if "success_step" in report and step - report["success_step"] >= round(AFTER_DONE_S / DT):
            break
        if "script_done_step" in report and step - report["script_done_step"] >= round(AFTER_DONE_S / DT):
            break
    st = world["states"]
    summary = {"variant": variant,
               "task_score": 1.0 if "success_step" in report else (float(result.info.get("score", 0.0)) if result else 0.0),
               "success_step": report.get("success_step"), "hooked": task.hooked(st),
               "steps": len(log), "seated": task.seated(st), "aligned": task.aligned(st),
               "eye_off_pull_deg": task.eye_off_pull_deg(st), "report": report, "wall_s": time.time() - t0}
    out.mkdir(parents=True, exist_ok=True)
    (out / "result.json").write_text(json.dumps(summary, indent=2, default=float))
    with open(out / "steps.jsonl", "w") as f:
        for row in log:
            f.write(json.dumps(row, default=float) + "\n")
    if video:
        write_videos(frames, out, variant)
    return summary


def write_videos(frames: dict, out: Path, variant: str) -> None:
    import cv2

    for cam, imgs in frames.items():
        if not imgs:
            continue
        h, w = imgs[0].shape[:2]
        raw = out / f"_{cam}.avi"
        writer = cv2.VideoWriter(str(raw), cv2.VideoWriter_fourcc(*"MJPG"), CONTROL_HZ, (w, h))
        for im in imgs:
            writer.write(np.ascontiguousarray(im[..., ::-1]))
        writer.release()
        name = out / f"demo_{cam}.mp4"
        if shutil.which("ffmpeg"):
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(raw), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                            "-movflags", "+faststart", str(name)], check=True)
            raw.unlink()
        else:
            raw.rename(name.with_suffix(".avi"))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", choices=VARIANTS, default="full")
    ap.add_argument("--out", type=Path, default=Path("demo_runs") / "full")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--max-steps", type=int, default=MAX_STEPS)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--no-abort", action="store_true")
    args = ap.parse_args(argv)
    global ABORT_EARLY
    ABORT_EARLY = not args.no_abort
    video = not args.no_video
    emb = make_embodiment(video, args.device)
    summary = run(args.variant, args.out, video, args.max_steps, emb)
    print("RESULT " + json.dumps({k: summary[k] for k in ("variant", "task_score", "success_step", "steps", "seated",
                                                           "aligned", "hooked", "eye_off_pull_deg", "wall_s")},
                                 default=float))
    sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
