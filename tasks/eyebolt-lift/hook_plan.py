"""Offline 2-D plan of how the hook goes into the eye (no simulator): in the hook's plane (the plane through the eye's
axis across the eye; the eye's sections there are discs), search from the hooked pose for a collision-free way out
by moving and tilting the hook (latch held open, so left out). Run backwards, that is the way in.

    python hook_plan.py            # prints the path (hook origin u, w in mm from the eye's top section; tilt deg)

Frame: u horizontal in the hook's plane, w up; the eye's top section's centre at (0, 0). A hook pose: its origin (its
top joint) at (u, w), tilted by a (degrees, counter-clockwise) from upright; hook-local x maps to +u when upright.
"""

from __future__ import annotations

import json
import math
import sys
import types
from collections import deque
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
if "yam_arena" not in sys.modules:  # el_parts imports yam_arena.task_file only for its base classes
    from dataclasses import dataclass

    ya, tf = types.ModuleType("yam_arena"), types.ModuleType("yam_arena.task_file")

    @dataclass(frozen=True)
    class Box:
        name: str = ""
        pos: tuple = (0, 0, 0)
        size: tuple = (0, 0, 0)
        mass: float | None = None
        color: tuple = (0, 0, 0)
        yaw: float = 0.0

        def _check(self):
            pass

    @dataclass(frozen=True)
    class ObjectState:
        pos: object = None
        quat_xyzw: object = None
        lin_vel: object = None
        ang_vel: object = None
        low: object = None
        high: object = None

    tf.Box, tf.ObjectState, ya.task_file = Box, ObjectState, tf
    sys.modules["yam_arena"], sys.modules["yam_arena.task_file"] = ya, tf
import el_parts as el  # noqa: E402

MM = 1e-3
CLEAR = 3.0  # mm of clearance required on the way in (0.6 planned a way the arm couldn't follow, demo runs 25-29) ...
NEAR = 1.0  # ... and in the bowl, where the section comes to rest (its free play there is less than CLEAR)
TILT_MAX = 15  # degrees from TILT0: the least band that finds a way is used (the arm's hand tilts with the hook)
TILT0 = 20  # the hooked pose's tilt: a hand taking the hanging hook from above holds it ~20 degrees tilted (its bowl
# hangs to one side), so a way in round 20 degrees keeps the hand upright (planned round 0, the hand had to tilt 20
# degrees the other way near the end, out of reach: demo runs 33-37). Hooked, any tilt rests the section in the bowl
RS = el.RING_RS / MM


def hook_segments():
    """The hook's rods in its plane (x, z in mm, hook frame), each (p0, p1); radius HOOK_W."""
    segs = []
    for sh in el.hook_shapes_local():
        pts = [(p[0] / MM, p[2] / MM) for p in sh.hull]
        n = len(pts) // 2
        c0 = (sum(p[0] for p in pts[:n]) / n, sum(p[1] for p in pts[:n]) / n)
        c1 = (sum(p[0] for p in pts[n:]) / n, sum(p[1] for p in pts[n:]) / n)
        segs.append((c0, c1))
    segs.append(((0.0, 0.0), (0.0, 12.0)))  # the first chain link's end, above the hook's joint
    return segs


SEGS = hook_segments()
R_HOOK = el.HOOK_W / MM
# the eye in this plane: its top section (where the hook will hang), its bottom section, the neck, the collar, the
# housing's top (relative to the top section's centre, mm)
RC = el.RING_RC / MM
TIP_TO_TOP = (el.RING_Z + el.RING_RC) / MM  # the top section's centre above the eyebolt's tip
DISCS = [((0.0, 0.0), RS), ((0.0, -2 * RC), RS)]
collar_top = (el.THREAD_LEN + el.COLLAR_H) / MM - TIP_TO_TOP
face = el.THREAD_LEN / MM - TIP_TO_TOP
BOXES = [(-6.0, 6.0, collar_top, -2 * RC), (-el.COLLAR_R / MM, el.COLLAR_R / MM, face, collar_top),
         (-200.0, 200.0, face - 50.0, face)]


def seg_point_dist(p, a, b):
    ax, az = a
    bx, bz = b
    dx, dz = bx - ax, bz - az
    L = dx * dx + dz * dz
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - az) * dz) / L))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (az + t * dz))


def seg_box_dist(a, b, box):
    x0, x1, z0, z1 = box
    best = math.inf
    for k in range(11):
        t = k / 10
        p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        dx = max(x0 - p[0], 0.0, p[0] - x1)
        dz = max(z0 - p[1], 0.0, p[1] - z1)
        best = min(best, math.hypot(dx, dz))
    return best


BOWL = (el.HOOK_BOWL_C[0] / MM, el.HOOK_BOWL_C[1] / MM)  # the pose's reference point (hook frame): the bowl's centre


def world_segs(u, w, a_deg):
    """The hook's rods with its bowl's centre at (u, w), tilted by a about it."""
    a = math.radians(a_deg)
    c, s = math.cos(a), math.sin(a)
    out = []
    for p0, p1 in SEGS:
        q0 = (p0[0] - BOWL[0], p0[1] - BOWL[1])
        q1 = (p1[0] - BOWL[0], p1[1] - BOWL[1])
        out.append(((u + c * q0[0] - s * q0[1], w + s * q0[0] + c * q0[1]),
                    (u + c * q1[0] - s * q1[1], w + s * q1[0] + c * q1[1])))
    return out


def origin_of(u, w, a_deg):
    """The hook's origin (its top joint) for a pose."""
    a = math.radians(a_deg)
    c, s = math.cos(a), math.sin(a)
    q = (-BOWL[0], -BOWL[1])
    return (u + c * q[0] - s * q[1], w + s * q[0] + c * q[1])


def clearance(u, w, a_deg):
    best = math.inf
    for a, b in world_segs(u, w, a_deg):
        for c, r in DISCS:
            best = min(best, seg_point_dist(c, a, b) - r - R_HOOK)
        for box in BOXES:
            best = min(best, seg_box_dist(a, b, box) - R_HOOK)
    return best


def free(u, w, a_deg):
    segs = world_segs(u, w, a_deg)
    pts = [p for s in segs for p in s]
    far = min(seg_point_dist((0.0, 0.0), a, b) for a, b in segs) > 10.0
    return far and (all(p[0] > 12 for p in pts) or all(p[0] < -12 for p in pts) or all(p[1] > 12 for p in pts))


IN_BOWL = (el.HOOK_BOWL_R - el.HOOK_W - el.RING_RS) / MM  # the section's free play in the bowl


def required(s):
    return NEAR if math.hypot(s[0], s[1]) <= IN_BOWL - NEAR else CLEAR


def plan(tilt_max=TILT_MAX):
    start = (0, 0, TILT0)  # the bowl's centre on the section's centre, tilted TILT0 (resting, it touches the bowl)
    if clearance(*start) < NEAR:
        print("hooked pose collides", clearance(*start))
    prev = {start: None}
    q = deque([start])
    goal = None
    steps = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 5), (0, 0, -5)]
    while q:
        s = q.popleft()
        if free(*s):
            goal = s
            break
        for du, dw, da in steps:
            n = (s[0] + du, s[1] + dw, s[2] + da)
            if n in prev or not (-80 <= n[0] <= 80 and -40 <= n[1] <= 120 and abs(n[2] - TILT0) <= tilt_max):
                continue
            if clearance(*n) < required(n):
                prev[n] = "x"
                continue
            prev[n] = s
            q.append(n)
    if goal is None:
        return None, len(prev)
    path = [goal]
    while prev[path[-1]] is not None:
        path.append(prev[path[-1]])
    return path, len(prev)  # from free to hooked: the way in


def keyframes(path, every=8):
    """Thin the path: keep turns in direction and every Nth state."""
    out = [path[0]]
    for i in range(1, len(path) - 1):
        d0 = tuple(b - a for a, b in zip(path[i - 1], path[i], strict=True))
        d1 = tuple(b - a for a, b in zip(path[i], path[i + 1], strict=True))
        if d0 != d1 or i % every == 0:
            out.append(path[i])
    out.append(path[-1])
    return out


if __name__ == "__main__":
    for cap in range(0, TILT_MAX + 1, 5):
        path, n = plan(cap)
        if path is not None:
            break
    if path is None:
        print(f"NO PATH ({n} states)")
        raise SystemExit(1)
    kf = keyframes(path)
    print(f"path: {len(path)} states, {len(kf)} keyframes, explored {n}; tilt range "
          f"{min(p[2] for p in path)}..{max(p[2] for p in path)} deg")
    for p in kf:
        o = origin_of(*p)
        print(p, "origin", tuple(round(v, 1) for v in o), "clear", round(clearance(*p), 2))
    (HERE / "hook_path.json").write_text(json.dumps({"frame": "bowl centre u,w mm from the eye's top section; tilt deg about it",
                                                     "path_in": path, "keyframes": kf}))
