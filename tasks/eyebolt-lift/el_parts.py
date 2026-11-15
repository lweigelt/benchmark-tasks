"""The eyebolt-lift task's own parts: a machine part (a gearbox housing, simplified to a cast-iron block) with a tapped
M10 hole, a loose M10 lifting eyebolt standing in a tray, and a chain sling hanging from a fixed overhead anchor with a
latched hook on its end. All geometry is our own simplified design (eyebolt proportions after DIN 580, to verify);
no manufacturer CAD is used.

yam_arena task files support single primitives and library assets only, so this module is the task's own minimal
mechanism (the approach of rail-plate-mount's rp_parts.py, copied so this folder stands alone):

- :class:`Part` is a :class:`yam_arena.task_file.Box` that also names which part it is.
- :func:`install` wraps ``yam_arena.scene.task_object`` and ``object_states`` the moment that module is imported.

The thread. As in rail-plate-mount, the eyebolt exists twice: the loose rigid eyebolt (kind "eyebolt") and its
threaded twin (kind "thread": anchor -> thread slider coupled to the turning by a mimic joint, 1.5 mm per turn ->
clamp spring under the shoulder -> the turning eyebolt). When the loose eyebolt stands upright in the hole and is
turned clockwise, the twin takes its place. **New here: the thread has a fixed start.** The twin's anchor sits at the
hole, turned to the hole's thread phase; the twin's joints are set so the eyebolt keeps its angle and its depth snaps
onto the helix (by at most half a pitch). So the eye's angle at shoulder contact is fixed by the hole, as with a real
thread, and the eye only lines up with the pull by tightening further past contact.

The sling: a world-fixed articulation hanging from the anchor: CHAIN_N rigid links, each joined to the next by two
revolute joints (a small gimbal body between them: a universal joint), then the hook, and on the hook a sprung latch
gate (a revolute joint, closed at its limit, opening inward into the hook's bowl).

Frames. Housing: the centre of its top face (z up). Eyebolt: the centre of its thread's tip, its axis +z, the eye in
its x-z plane. Thread twin: the anchor at the hole, turned to the thread phase. Tray: the centre of its top face.
Sling: the anchor point; the chain hangs along -z; the hook's plane is its x-z plane, the throat opening on its -x side.
"""

from __future__ import annotations

import importlib.abc
import importlib.machinery
import inspect
import math
import os
import sys
import tempfile
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

from yam_arena import task_file

MM = 1e-3
# the size: M16 (default; the hook and eye sized so the hook goes in with ~3 mm to spare, design notes "M16") or M10
# (the first build: ~1 mm to spare, too tight for the arm, demo runs 7-29)
SIZE = os.environ.get("EL_SIZE", "M16")
_M16 = SIZE == "M16"

# -- machine part: a gearbox housing, simplified to a block, fixed ----------------------------------------------
HOUSING_SIZE = (160 * MM, 120 * MM, (40 if _M16 else 60) * MM)  # x, y, z (its top at the origin; low enough that the arms reach the eye
# pointing down: they reach only ~12-16 cm above the table that way)
HOUSING_HOLES = [(0.0, 0.0)]  # tapped M10 holes in the top face (x, y); a bench may set several
TOP_T = (32.0 if _M16 else 25.0) * MM  # the top wall the holes are cut through (deeper than the eyebolt's thread)
HOLE_HALF = (7.0 if _M16 else 4.25) * MM  # the tapped hole's core (M16: 14 mm; M10: 8.5 mm), as an octagon ...
HOLE_SINK_HALF = (8.5 if _M16 else 5.5) * MM  # ... with a 90 degree countersink at the top (as tapped holes are made)
HOLE_CUT = (11.0 if _M16 else 8.0) * MM  # the square cut-out round each hole, filled with the eight countersunk wall pieces

# -- eyebolt (DIN 580 proportions: to verify; M16: eye 63 mm outside, 35 inside, 14 section, collar 35, thread 27) ----
PITCH = (2.0 if _M16 else 1.5) * MM
THREAD_LEN = (27.0 if _M16 else 17.0) * MM  # under the shoulder to the tip
DOG_LEN, DOG_R = 1.0 * MM, (6.6 if _M16 else 3.9) * MM  # the thread's chamfered start (what centres it in the hole)
SHANK_R = (7.9 if _M16 else 4.9) * MM  # loose eyebolt: the thread's outside (sits on the countersink)
THREAD_SHANK_R = (6.8 if _M16 else 4.0) * MM  # threaded twin: passes through the hole
SEAT_DEPTH = DOG_LEN + (HOLE_SINK_HALF - SHANK_R)  # standing loose in the hole: its tip this far below the top
COLLAR_R, COLLAR_H = (17.5 if _M16 else 12.5) * MM, (8.0 if _M16 else 6.0) * MM  # the shoulder (collar) above the thread
BEARING_N, BEARING_RING, BEARING_R = 12, (13.0 if _M16 else 9.0) * MM, 0.5 * MM  # the shoulder bears on small balls
NECK_R = (9.0 if _M16 else 6.0) * MM
RING_RC, RING_RS = (24.5 * MM, 7.0 * MM) if _M16 else (17.5 * MM, 5.0 * MM)
# the eye: centreline radius, section radius (M16: 63 outside, 35 inside; M10: 45, 25)
RING_Z = THREAD_LEN + COLLAR_H + RING_RC + RING_RS - 3.0 * MM  # the eye's centre above the tip
RING_SEGMENTS = 16
EYEBOLT_MASS = 0.28 if _M16 else 0.105  # kg
EYEBOLT_ARMATURE = 1.0e-4  # kg m^2 on the twin's spin joint (a hand's share; the eyebolt's own is ~3e-5)

# -- thread twin ---------------------------------------------------------------------------------------------------
THREAD_PHASE = math.radians(75.0)  # the eye's angle at shoulder contact, from the pull direction (counter-clockwise
# from above: tightening, clockwise, turns it toward the pull). Set per hole by the task from its pull direction.
CLAMP_SPRING = 1.0e5  # N/m: the shoulder's clamp (the joint's elasticity)
CLAMP_TRAVEL = 0.8 * MM  # hard stop: ~190 degrees past contact (the alignment needs up to ~90)
CONTACT_CLAMP = 2.0  # N: the shoulder counts as touching once the clamp carries this ...
CONTACT_STEPS = 10  # ... for this many control steps in a row
FREE_TORQUE = 0.05  # Nm: thread friction (constant)
THREAD_RANGE = THREAD_LEN + 8.0 * MM  # the thread slider's travel: beyond the whole thread (25 mm fixed was short of M16's 27: catches near the limit were released at once, thread benches 3-4)
LINK_ARMATURE = 0.001
LINK_MASS = 0.0005
ANCHOR_MASS = 0.001

# -- catching the thread (as rail-plate-mount) ------------------------------------------------------------------------
ENGAGE_R = 1.2 * MM
ENGAGE_R_HYST = 0.4 * MM
ENGAGE_DZ = 0.3 * MM
ENGAGE_MIN_DEPTH = 0.3 * MM
ENGAGE_TILT = math.radians(3.0)
START_ANGLE = math.radians(20.0)
BACKOUT_TURNS = 0.25
PARK = (-1.0, 0.0, 0.05)

# -- tray (the eyebolt stands in it, thread down, its shoulder on the top) --------------------------------------
TRAY_SIZE = (60 * MM, 60 * MM, 32 * MM) if _M16 else (50 * MM, 50 * MM, 25 * MM)
TRAY_HOLE_HALF = (8.6 if _M16 else 5.6) * MM
TRAY_MASS = 0.35

# -- sling: chain from a fixed anchor, hook with a latch -------------------------------------------------------
CHAIN_N = 16  # 48 cm from an anchor 55 cm up: the hook hangs 12 mm above the table, ~5 cm slack when hooked (8 links
# left the chain taut and the hook short of the eye, hook bench 1; with the anchor 40 cm up, its bracket stopped the
# right arm reaching down to the hook, demo runs 16-18)
CHAIN_LINK = 30.0 * MM  # joint to joint
CHAIN_R = 2.0 * MM  # a link's section (a rod; a real link is an oval ring): 4 mm chain
CHAIN_LINK_MASS = 0.01  # 4 mm chain: ~0.35 kg/m
CHAIN_DAMPING = 1.0e-3  # Nm s/rad on each chain joint (quiets the hanging chain)
CHAIN_COLLIDE = os.environ.get("EL_CHAIN_COLLIDE", "0") != "0"  # simulation: the chain's links don't collide (only
# the hook and its latch do). A hand taking the hook from above meets the chain hanging over it with its base; a real
# chain gives way, ours (rigid rods on universal joints) swung the hook away and the grip missed (demo runs 7-21)
GIMBAL_MASS = 0.002
# the hook: sized for 4 mm chain and the M10 eye (its 10 mm section must pass the throat and rest in the bowl; a
# larger hook's bowl couldn't pass through the eye's 25 mm hole, design notes)
# M16: a 22 mm throat leaves ~3 mm round the eye's 14 mm section on the way in, tilting the hook at most 15 degrees;
# its tip 12 mm above the bowl's centre and the latch hinged 13 mm above that let the latch close over the section
# resting in the bowl (a lower tip needed less tilt, but the opened latch then swept the resting section: hook bench 9;
# offline sweeps, design notes "M16")
HOOK_MASS = 0.12 if _M16 else 0.05
LATCH_MASS = 0.005 if _M16 else 0.003
HOOK_W = (4.0 if _M16 else 3.5) * MM  # half the hook's section
HOOK_SHANK_TOP, HOOK_SHANK_BOT = -5.0 * MM, -44.0 * MM  # the shank, x in [-HOOK_W, HOOK_W]
HOOK_BOWL_R = (15.0 if _M16 else 10.0) * MM
HOOK_BOWL_C = (-HOOK_BOWL_R, -44.0 * MM)  # the bowl's centre (x, z) (it meets the shank's bottom)
HOOK_TIP_TOP = (-32.0 if _M16 else -26.0) * MM  # the tip rises at x = -2R to here (centreline)
# the latch gate: from its hinge on the shank's inner face down to the tip's top (M16: slanting, so that opened it
# leaves the 14 mm section room and still clears the bowl; M10: level)
LATCH_HINGE = (-HOOK_W, (-19.0 if _M16 else -24.0) * MM)
LATCH_END = ((-25.0, -27.0) if _M16 else (-17.0, -24.0))
LATCH_END = (LATCH_END[0] * MM, LATCH_END[1] * MM)
LATCH_LEN = math.hypot(LATCH_END[0] - LATCH_HINGE[0], LATCH_END[1] - LATCH_HINGE[1])
LATCH_OPEN = math.radians(54.0 if _M16 else 80.0)  # it opens inward (down into the bowl) up to this
LATCH_SPRING = 0.01  # Nm/rad: closes it
LATCH_DAMPING = 1.0e-5

# -- materials and looks ------------------------------------------------------------------------------------------
MATERIALS = {  # (static friction, dynamic friction, combine mode)
    "steel": (0.30, 0.30, "average"),
    "screw": (0.20, 0.20, "average"),
    "cast": (0.35, 0.35, "average"),
}
LOOKS = {  # sRGB, roughness, metallic
    "steel": ((0.72, 0.74, 0.76), 0.35, 0.8),
    "zinc": ((0.80, 0.80, 0.78), 0.3, 0.9),
    "cast": ((0.30, 0.42, 0.55), 0.6, 0.2),  # painted housing
    "hole": ((0.03, 0.03, 0.035), 0.9, 0.0),
    "chain": ((0.85, 0.65, 0.10), 0.4, 0.6),  # grade-80 chain is often painted yellow/orange
    "hook": ((0.85, 0.65, 0.10), 0.4, 0.6),
    "latch": ((0.75, 0.76, 0.78), 0.3, 0.9),
    "black": ((0.08, 0.08, 0.09), 0.7, 0.0),
}
CONTACT_OFFSET = 0.5 * MM


@dataclass
class Shape:
    """One piece of a part, in its body's frame: a box (``center``, ``size``), a prism (a 2-D ``profile``
    extruded over ``span``: (x, z) along y for "xz", (x, y) along z for "xy", (y, z) along x for "yz"), a convex hull
    of points (``hull``), a sphere (``axis`` "S") or a visual cylinder (``center``, ``radius``, ``length``)."""

    body: str
    look: str
    material: str | None  # None: visual only
    center: tuple[float, float, float] = (0.0, 0.0, 0.0)
    size: tuple[float, float, float] | None = None
    profile: tuple[tuple[float, float], ...] | None = None
    span: tuple[float, float] = (0.0, 0.0)
    plane: str = "xz"
    radius: float = 0.0
    length: float = 0.0
    axis: str = "Z"
    visible: bool = True
    hull: tuple | None = None

    def lift(self, p, s):
        if self.plane == "xz":
            return (p[0], s, p[1])
        if self.plane == "yz":
            return (s, p[0], p[1])
        return (p[0], p[1], s)

    def points(self):
        if self.hull is not None:
            return list(self.hull)
        if self.profile is not None:
            return [self.lift(p, s) for p in self.profile for s in self.span]
        if self.size is not None:
            return [tuple(c + s * h / 2 for c, h, s in zip(self.center, self.size, sg, strict=True))
                    for sg in ((-1, -1, -1), (1, 1, 1))]
        r, h = self.radius, self.length / 2
        ext = {"X": (h, r, r), "Y": (r, h, r), "Z": (r, r, h), "S": (r, r, r)}[self.axis]
        return [tuple(c + s * e for c, e in zip(self.center, ext, strict=True)) for s in (-1, 1)]


def _box(body, look, material, x0, x1, y0, y1, z0, z1, visible=True) -> Shape:
    return Shape(body, look, material, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0),
                 visible=visible)


def _prism(body, look, material, profile, s0, s1, plane="xz", visible=True) -> Shape:
    return Shape(body, look, material, profile=tuple((float(a), float(b)) for a, b in profile), span=(s0, s1),
                 plane=plane, visible=visible)


def _ngon(r, n, start=0.0, a0=0.0, a1=2 * math.pi):
    full = abs(a1 - a0 - 2 * math.pi) < 1e-9
    k = n if full else n + 1
    return [(r * math.cos(a0 + start + (a1 - a0) * i / n), r * math.sin(a0 + start + (a1 - a0) * i / n))
            for i in range(k)]


def _tube(body, look, material, p0, p1, r, n=8, visible=True) -> Shape:
    """A convex rod of octagonal section from p0 to p1 (3-D points), as a hull (visible: also drawn)."""
    import numpy as np

    a, b = np.asarray(p0, float), np.asarray(p1, float)
    d = b - a
    d /= np.linalg.norm(d)
    u = np.cross(d, (0.0, 0.0, 1.0)) if abs(d[2]) < 0.9 else np.cross(d, (1.0, 0.0, 0.0))
    u /= np.linalg.norm(u)
    v = np.cross(d, u)
    pts = []
    for c in (a, b):
        for k in range(n):
            t = 2 * math.pi * (k + 0.5) / n
            pts.append(tuple(float(x) for x in c + r * (math.cos(t) * u + math.sin(t) * v)))
    return Shape(body, look, material, hull=tuple(pts), visible=visible)


# -- housing ------------------------------------------------------------------------------------------------------


def hole_pieces(body, cx: float, cy: float, t: float) -> list[Shape]:
    """The wall round a tapped hole in a wall (top at z 0, ``t`` thick) inside a square cut-out: eight convex
    countersunk pieces round an octagon (as rail-plate-mount's plate holes)."""
    h, hs, cut = HOLE_HALF, HOLE_SINK_HALF, HOLE_CUT
    sink = hs - h
    r_v = h / math.cos(math.pi / 8)
    out = []
    for k in range(8):
        a0, a1 = (k - 0.5) * math.pi / 4, (k + 0.5) * math.pi / 4
        inner = [(r_v * math.cos(a), r_v * math.sin(a)) for a in (a0, a1)]
        ring = []
        for a in (a0, a1):
            c, s_ = math.cos(a), math.sin(a)
            m = cut / max(abs(c), abs(s_))
            ring.append((m * c, m * s_))
        for sx in (1, -1):
            for sy in (1, -1):
                ac = math.atan2(sy, sx)
                if any(a0 < ac + n * 2 * math.pi < a1 for n in (-1, 0, 1)):
                    ring.append((sx * cut, sy * cut))
        pts = []
        for x, y in ring:
            pts += [(x, y, -t), (x, y, 0.0)]
        for x, y in inner:
            pts += [(x, y, -t), (x, y, -sink), (x * hs / h, y * hs / h, 0.0)]
        out.append(Shape(body, "cast", "cast", hull=tuple((cx + x, cy + y, z) for x, y, z in pts), visible=False))
    for x0, x1, y0, y1 in ((-cut, cut, -cut, -hs), (-cut, cut, hs, cut), (-cut, -hs, -hs, hs), (hs, cut, -hs, hs)):
        out.append(_box(body, "cast", None, x0 + cx, x1 + cx, y0 + cy, y1 + cy, -t, 0.0))
    out.append(Shape(body, "hole", None, (cx, cy, -t / 2 - 0.05 * MM), radius=h * 0.98, length=t))
    out.append(Shape(body, "zinc", None, (cx, cy, -sink / 2), radius=(h + hs) / 2, length=sink))
    return out


def housing_shapes() -> list[Shape]:
    """The block (origin: the centre of its top face) with its top wall cut round each hole."""
    ax, ay, hz = HOUSING_SIZE[0] / 2, HOUSING_SIZE[1] / 2, HOUSING_SIZE[2]
    cut, t = HOLE_CUT, TOP_T
    out = [_box("housing", "cast", "cast", -ax, ax, -ay, ay, -hz, -t)]
    xs = sorted({x for x, _ in HOUSING_HOLES})
    # the top wall: strips between the holes' columns, and in each column pieces between its holes
    edges = [-ax] + [v for x in xs for v in (x - cut, x + cut)] + [ax]
    for x0, x1 in zip(edges[::2], edges[1::2]):
        out.append(_box("housing", "cast", "cast", x0, x1, -ay, ay, -t, 0.0))
    for x in xs:
        ys = sorted(y for xx, y in HOUSING_HOLES if xx == x)
        yedges = [-ay] + [v for y in ys for v in (y - cut, y + cut)] + [ay]
        for y0, y1 in zip(yedges[::2], yedges[1::2]):
            out.append(_box("housing", "cast", "cast", x - cut, x + cut, y0, y1, -t, 0.0))
        for y in ys:
            out += hole_pieces("housing", x, y, t)
    return out


# -- eyebolt ---------------------------------------------------------------------------------------------------------


def eyebolt_shapes(body: str = "eyebolt", shank_r: float = SHANK_R) -> list[Shape]:
    """Thread (a dog point and the shank), the collar (shoulder) bearing on small balls, the neck and the eye: a ring
    in the x-z plane of RING_SEGMENTS convex segments."""
    z0 = THREAD_LEN
    out = [_prism(body, "steel", "screw", _ngon(DOG_R, 12), 0.0, DOG_LEN, plane="xy"),
           _prism(body, "steel", "screw", _ngon(shank_r, 16), DOG_LEN, z0, plane="xy")]
    for k in range(BEARING_N):
        a = 2 * math.pi * (k + 0.5) / BEARING_N
        out.append(Shape(body, "steel", "screw", (BEARING_RING * math.cos(a), BEARING_RING * math.sin(a),
                                                   z0 + BEARING_R), radius=BEARING_R, axis="S", visible=False))
    out.append(_prism(body, "steel", None, _ngon(COLLAR_R, 24), z0, z0 + COLLAR_H, plane="xy"))  # the look
    out.append(_prism(body, "steel", "steel", _ngon(COLLAR_R, 24), z0 + 0.4 * MM, z0 + COLLAR_H, plane="xy",
                      visible=False))
    # the neck: from the collar up into the ring's bottom
    zr = RING_Z - RING_RC
    out.append(_prism(body, "steel", "steel", _ngon(NECK_R, 12), z0 + COLLAR_H, zr + 1.0 * MM, plane="xy"))
    for k in range(RING_SEGMENTS):
        t0, t1 = 2 * math.pi * k / RING_SEGMENTS, 2 * math.pi * (k + 1) / RING_SEGMENTS
        p0 = (RING_RC * math.sin(t0), 0.0, RING_Z - RING_RC * math.cos(t0))
        p1 = (RING_RC * math.sin(t1), 0.0, RING_Z - RING_RC * math.cos(t1))
        out.append(_tube(body, "steel", "steel", p0, p1, RING_RS))
    return out


def thread_shapes() -> list[Shape]:
    return eyebolt_shapes("eyebolt", THREAD_SHANK_R)


def ring_top_local():
    """The middle of the eye's top section (what a hook hangs on), in the eyebolt's frame."""
    return (0.0, 0.0, RING_Z + RING_RC)


# -- tray ------------------------------------------------------------------------------------------------------------


def tray_shapes() -> list[Shape]:
    hx, hy, hz = TRAY_SIZE[0] / 2, TRAY_SIZE[1] / 2, TRAY_SIZE[2]
    h = TRAY_HOLE_HALF
    return [_box("tray", "steel", "steel", -hx, -h, -hy, hy, -hz, 0.0),
            _box("tray", "steel", "steel", h, hx, -hy, hy, -hz, 0.0),
            _box("tray", "steel", "steel", -h, h, -hy, -h, -hz, 0.0),
            _box("tray", "steel", "steel", -h, h, h, hy, -hz, 0.0),
            _box("tray", "steel", "steel", -h, h, -h, h, -hz, -hz + 3 * MM),  # the hole's floor
            Shape("tray", "hole", None, (0.0, 0.0, -hz / 2), radius=h * 0.95, length=hz - 0.2 * MM)]


# -- sling -------------------------------------------------------------------------------------------------------------


def sling_bodies():
    """Bodies in hanging order with their frame's position in the sling's frame (the anchor at the origin)."""
    out = [("anchor", (0.0, 0.0, 0.0))]
    for i in range(CHAIN_N):
        z = -i * CHAIN_LINK
        out += [(f"g{i}", (0.0, 0.0, z)), (f"link{i}", (0.0, 0.0, z))]
    zh = -CHAIN_N * CHAIN_LINK
    out += [(f"g{CHAIN_N}", (0.0, 0.0, zh)), ("hook", (0.0, 0.0, zh)),
            ("latch", (LATCH_HINGE[0], 0.0, zh + LATCH_HINGE[1]))]
    return out


def sling_joints():
    """name: (parent, child, axis, lower deg, upper deg, stiffness, damping) (revolute; at the child's frame origin)."""
    out = {}
    parent = "anchor"
    for i in range(CHAIN_N + 1):
        child = f"link{i}" if i < CHAIN_N else "hook"
        out[f"c{i}x"] = (parent, f"g{i}", "X", -150.0, 150.0, 0.0, CHAIN_DAMPING)
        out[f"c{i}y"] = (f"g{i}", child, "Y", -150.0, 150.0, 0.0, CHAIN_DAMPING)
        parent = child
    out["latch"] = ("hook", "latch", "Y", -math.degrees(LATCH_OPEN), 0.0, LATCH_SPRING, LATCH_DAMPING)
    return out


def hook_shapes_local():
    """The hook's pieces in its own frame (origin: its top joint): eye, shank, bowl (8 segments), tip."""
    w = HOOK_W
    out = [_tube("hook", "hook", "steel", (0.0, 0.0, -2.0 * MM), (0.0, 0.0, HOOK_SHANK_TOP), w),
           _tube("hook", "hook", "steel", (0.0, 0.0, HOOK_SHANK_TOP), (0.0, 0.0, HOOK_SHANK_BOT), w)]
    bx, bz = HOOK_BOWL_C
    n = 8
    for k in range(n):
        t0, t1 = -math.pi * k / n, -math.pi * (k + 1) / n
        p0 = (bx + HOOK_BOWL_R * math.cos(t0), 0.0, bz + HOOK_BOWL_R * math.sin(t0))
        p1 = (bx + HOOK_BOWL_R * math.cos(t1), 0.0, bz + HOOK_BOWL_R * math.sin(t1))
        out.append(_tube("hook", "hook", "steel", p0, p1, w))
    xt = bx - HOOK_BOWL_R
    out.append(_tube("hook", "hook", "steel", (xt, 0.0, bz), (xt, 0.0, HOOK_TIP_TOP), w))
    return out


def latch_shape(zh: float = 0.0) -> Shape:
    """The latch gate: a flat bar 6 mm wide, 2 mm thick, from the hinge to its end (in the sling's frame, the hook's
    joint at z zh)."""
    (hx, hz), (ex, ez) = LATCH_HINGE, LATCH_END
    d = (ex - hx, ez - hz)
    n = math.hypot(*d)
    nx, nz = -d[1] / n * 1.0 * MM, d[0] / n * 1.0 * MM  # the bar's half thickness, across it in the plane
    pts = tuple((x + sx * nx, y, z + zh + sx * nz) for x, z in ((hx, hz), (ex, ez)) for sx in (-1, 1)
                for y in (-3.0 * MM, 3.0 * MM))
    return Shape("latch", "latch", "steel", hull=pts)


def sling_shapes() -> list[Shape]:
    out = [Shape("anchor", "black", None, (0.0, 0.0, 0.0), radius=8 * MM, axis="S")]
    for i in range(CHAIN_N):
        z = -i * CHAIN_LINK
        out.append(_tube(f"link{i}", "chain", "steel" if CHAIN_COLLIDE else None, (0.0, 0.0, z - 2.0 * MM),
                         (0.0, 0.0, z - CHAIN_LINK + 2.0 * MM), CHAIN_R))
    zh = -CHAIN_N * CHAIN_LINK
    for sh in hook_shapes_local():
        out.append(Shape(sh.body, sh.look, sh.material, hull=tuple((x, y, z + zh) for x, y, z in sh.hull)))
    out.append(latch_shape(zh))
    return out


# -- hook stand: the hook parks upright in it, its bowl in a slot (the chain runs off sideways to the anchor, so a hand
# can take the hook from above: hanging under the anchor, the chain above the hook was in the gripper's way) --------
STAND_SIZE = (40 * MM, 24 * MM, 35 * MM)  # x (along the slot), y, z
STAND_SLOT_HALF = 4.5 * MM  # a 9 mm slot along x for the hook's 7 mm section ...
STAND_SLOT_DEPTH = 30 * MM  # ... this deep (the bowl and the tip's lower half in it)
SLING_INIT: dict = {}  # the sling's starting joint angles (set by the task: chain_start())

GANTRY_H = 0.34  # the anchor's height above the table (set by the task)
GANTRY_REACH = 0.22  # the post stands this far behind the anchor (along +x; 10 cm behind, the right hand reaching down
# to the hanging hook ran into it, demo runs 19-20)


def gantry_shapes() -> list[Shape]:
    """A post with an arm (origin: the anchor point under the arm's end): a simple overhead bracket."""
    h, r = GANTRY_H, GANTRY_REACH
    m = None if os.environ.get("EL_GANTRY_COLLIDE", "1") == "0" else "steel"  # (diagnosis switch)
    return [_box("gantry", "black", m, r - 20 * MM, r + 20 * MM, -20 * MM, 20 * MM, -h, 30 * MM),
            _box("gantry", "black", m, -15 * MM, r, -15 * MM, 15 * MM, 10 * MM, 30 * MM),
            _box("gantry", "black", m, r - 60 * MM, r + 60 * MM, -60 * MM, 60 * MM, -h, -h + 10 * MM)]


def stand_shapes() -> list[Shape]:
    """A block (origin: the centre of its top face) with a slot along x the hook stands in."""
    hx, hy, hz = STAND_SIZE[0] / 2, STAND_SIZE[1] / 2, STAND_SIZE[2]
    h, d = STAND_SLOT_HALF, STAND_SLOT_DEPTH
    return [_box("stand", "black", "steel", -hx, hx, -hy, -h, -hz, 0.0),
            _box("stand", "black", "steel", -hx, hx, h, hy, -hz, 0.0),
            _box("stand", "black", "steel", -hx, hx, -h, h, -hz, -d)]


def chain_start(anchor, hook_origin):
    """Joint angles that lay the chain from the anchor to the hook's origin (both env-frame (x, y, z)) in a smooth
    sag, in the vertical plane through both, with the hook upright: link i's angle from hanging straight down runs
    linearly from a to b (solved so the chain's end lands on the hook's origin). Returns {joint: angle}."""
    dx, dy, dz = (h - a for h, a in zip(hook_origin, anchor, strict=True))
    horiz = math.hypot(dx, dy)
    phi = math.atan2(dy, dx)  # the plane's direction
    n, L = CHAIN_N, CHAIN_LINK

    def end(a, b):
        th = [a + (b - a) * i / (n - 1) for i in range(n)]
        return sum(L * math.sin(t) for t in th), -sum(L * math.cos(t) for t in th), th

    best = None
    for ia in range(-60, 181, 2):
        for ib in range(-60, 181, 2):
            h, v, th = end(math.radians(ia), math.radians(ib))
            err = math.hypot(h - horiz, v - dz)
            if best is None or err < best[0]:
                best = (err, th)
    th = best[1]
    # the joints: a hanging link swings toward +x when turned negatively about +y (R_y(t) takes (0,0,-1) to
    # (-sin t, 0, -cos t)) and toward +y when turned positively about +x; for the plane at phi, as two joints:
    # x angle theta*sin(phi), y angle -theta*cos(phi) (exact for phi a multiple of 90 degrees)
    out, prev = {}, 0.0
    for i in range(n):
        d = th[i] - prev
        out[f"c{i}x"] = d * math.sin(phi)
        out[f"c{i}y"] = -d * math.cos(phi)
        prev = th[i]
    out[f"c{n}x"] = -prev * math.sin(phi)  # the hook upright again
    out[f"c{n}y"] = prev * math.cos(phi)
    out["latch"] = 0.0
    return out, best[0]


SHAPES = {"housing": housing_shapes, "gantry": gantry_shapes, "stand": stand_shapes, "eyebolt": eyebolt_shapes, "thread": thread_shapes, "tray": tray_shapes,
          "sling": sling_shapes}
BODIES = {"housing": ("housing",), "gantry": ("gantry",), "stand": ("stand",), "eyebolt": ("eyebolt",), "tray": ("tray",),
          "thread": ("anchor", "carrier", "clamp", "eyebolt"),
          "sling": tuple(b for b, _ in sling_bodies())}
MASSES = {"eyebolt": EYEBOLT_MASS, "tray": TRAY_MASS, "anchor": ANCHOR_MASS, "carrier": LINK_MASS, "clamp": LINK_MASS,
          "hook": HOOK_MASS, "latch": LATCH_MASS,
          **{f"link{i}": CHAIN_LINK_MASS for i in range(CHAIN_N)}, **{f"g{i}": GIMBAL_MASS for i in range(CHAIN_N + 1)}}


@lru_cache(maxsize=None)
def bounds(kind: str):
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    for sh in SHAPES[kind]():
        for p in sh.points():
            for i in range(3):
                lo[i], hi[i] = min(lo[i], p[i]), max(hi[i], p[i])
    return tuple(lo), tuple(hi)


@lru_cache(maxsize=None)
def local_center(kind: str):
    lo, hi = bounds(kind)
    return tuple((a + b) / 2 for a, b in zip(lo, hi, strict=True))


def _triangulate(poly):
    idx = list(range(len(poly)))
    tris = []

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def inside(p, a, b, c):
        return cross(a, b, p) >= 0 and cross(b, c, p) >= 0 and cross(c, a, p) >= 0

    guard = 0
    while len(idx) > 3 and guard < 100000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if cross(a, b, c) <= 1e-14:
                continue
            if any(inside(poly[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            raise ValueError("polygon is not simple or not counter-clockwise")
    tris.append(tuple(idx))
    return tris


def polygon_area(poly) -> float:
    return 0.5 * sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(poly, list(poly[1:]) + [poly[0]]))


def _hull_mesh(pts):
    """A closed triangle mesh of the convex hull of 3-D points (for the look of hull pieces)."""
    import numpy as np

    p = np.asarray(pts, float)
    n = len(p)
    faces = []
    c = p.mean(axis=0)
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                nrm = np.cross(p[j] - p[i], p[k] - p[i])
                ln = np.linalg.norm(nrm)
                if ln < 1e-12:
                    continue
                nrm /= ln
                d = (p - p[i]) @ nrm
                if np.all(d <= 1e-9):
                    faces.append((i, j, k) if np.dot(c - p[i], nrm) < 0 else (i, k, j))
                elif np.all(d >= -1e-9):
                    faces.append((i, k, j) if np.dot(c - p[i], nrm) > 0 else (i, j, k))
    return faces


# -- task-file objects ------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Part(task_file.Box):
    """One of this module's parts as a task-file object: ``pos`` is the part's origin, ``roll``/``pitch`` turns
    about the world x/y axes applied before ``yaw``."""

    kind: str = "eyebolt"
    roll: float = 0.0
    pitch: float = 0.0

    def _check(self) -> None:
        super()._check()
        if self.kind not in SHAPES:
            raise ValueError(f"{self.name}: unknown part kind {self.kind!r}")


def part(name, kind, pos, yaw=0.0, roll=0.0, pitch=0.0, fixed=False) -> Part:
    lo, hi = bounds(kind)
    mass = None if fixed or kind in ("housing", "gantry", "stand", "thread", "sling") else MASSES[kind]
    color = {"housing": LOOKS["cast"][0], "sling": LOOKS["chain"][0], "gantry": LOOKS["black"][0]}.get(
        kind, LOOKS["steel"][0])
    size = tuple(b - a for a, b in zip(lo, hi, strict=True))
    return Part(name=name, pos=tuple(float(v) for v in pos), size=size, mass=mass, color=color, yaw=float(yaw),
                kind=kind, roll=float(roll), pitch=float(pitch))


def twin_name(bolt: str) -> str:
    return f"{bolt}_thread"


def eyebolt_parts(name: str, pos, park_y: float, yaw=0.0, roll=0.0, pitch=0.0, fixed=False) -> list[Part]:
    """A loose eyebolt and its parked threaded twin (a fixed eyebolt: no twin)."""
    if fixed:
        return [part(name, "eyebolt", pos, yaw=yaw, roll=roll, pitch=pitch, fixed=True)]
    return [part(name, "eyebolt", pos, yaw=yaw, roll=roll, pitch=pitch),
            part(twin_name(name), "thread", (PARK[0], park_y, PARK[2]))]


def part_quat_xyzw(spec) -> tuple[float, float, float, float]:
    def q(axis, a):
        s = math.sin(a / 2)
        return (s * (axis == 0), s * (axis == 1), s * (axis == 2), math.cos(a / 2))

    return _qmul(q(2, spec.yaw), _qmul(q(1, spec.pitch), q(0, spec.roll)))


def _qmul(p, r):
    px, py, pz, pw = p
    rx, ry, rz, rw = r
    return (pw * rx + px * rw + py * rz - pz * ry, pw * ry - px * rz + py * rw + pz * rx,
            pw * rz + px * ry - py * rx + pz * rw, pw * rw - px * rx - py * ry - pz * rz)


@dataclass(frozen=True)
class BoltState(task_file.ObjectState):
    """An eyebolt's ``ObjectState`` (of whichever body is live) plus its thread: ``engaged``, ``hole`` (index, -1 if
    loose), ``turns`` (since it caught), ``clamp_force`` (N), ``past_contact`` (turns since the shoulder touched),
    ``gap`` (m: the shoulder's height above the hole's top face, along the hole's axis; loose: None)."""

    engaged: bool = False
    hole: int = -1
    turns: float = 0.0
    clamp_force: float = 0.0
    past_contact: float = 0.0
    gap: float | None = None
    touching: bool = False


@dataclass(frozen=True)
class SlingState(task_file.ObjectState):
    """The sling's ``ObjectState`` is its hook's (pos: the hook's frame origin, its top joint); plus ``latch`` (rad,
    0 closed, negative open) and the anchor's position."""

    latch: float = 0.0
    anchor: tuple = (0.0, 0.0, 0.0)
    hand_dist: float = float("inf")  # m: the nearest robot fingertip body to the hook's collider points


# -- USD ------------------------------------------------------------------------------------------------------------------

_USD_DIR: Path | None = None


def _srgb_to_linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _apply(prim, schema: str, attrs: dict) -> None:
    from pxr import Sdf

    prim.AddAppliedSchema(schema)
    types = {bool: Sdf.ValueTypeNames.Bool, int: Sdf.ValueTypeNames.Int, float: Sdf.ValueTypeNames.Float,
             str: Sdf.ValueTypeNames.Token}
    for name, value in attrs.items():
        prim.CreateAttribute(name, types[type(value)]).Set(value)


def _prism_mesh(sh: Shape):
    prof = list(sh.profile)
    if polygon_area(prof) < 0:
        prof = prof[::-1]
    n = len(prof)
    s0, s1 = sh.span
    pts = [sh.lift(p, s0) for p in prof] + [sh.lift(p, s1) for p in prof]
    counts, indices = [], []
    for a, b, c in _triangulate(prof):
        counts += [3, 3]
        indices += [a, b, c, n + c, n + b, n + a]
    for i in range(n):
        j = (i + 1) % n
        counts.append(4)
        indices += [i, n + i, n + j, j]
    return pts, counts, indices


def write_usd(kind: str, path: Path, fixed: bool = False) -> Path:
    """One part as a USD file: bodies, colliders, looks, materials and (for the thread twin and the sling) joints."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Part").GetPrim()
    stage.SetDefaultPrim(root)

    looks, mats = {}, {}
    for name, (rgb, rough, metal) in LOOKS.items():
        mat = UsdShade.Material.Define(stage, f"/Part/Looks/{name}")
        shader = UsdShade.Shader.Define(stage, f"/Part/Looks/{name}/shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(_srgb_to_linear, rgb)))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        looks[name] = mat
    for name, (static, dynamic, combine) in MATERIALS.items():
        mat = UsdShade.Material.Define(stage, f"/Part/PhysicsMaterials/{name}")
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0.0)
        _apply(mat.GetPrim(), "PhysxMaterialAPI", {"physxMaterial:frictionCombineMode": combine})
        mats[name] = mat

    jointed = kind in ("thread", "sling")
    frames = dict(sling_bodies()) if kind == "sling" else {}
    body_paths = {}
    for body in BODIES[kind]:
        path_b = f"/Part/{body}" if jointed else "/Part"
        xf = UsdGeom.Xform.Define(stage, path_b) if jointed else UsdGeom.Xform(root)
        prim = xf.GetPrim()
        if jointed and body in frames and any(frames[body]):
            xf.AddTranslateOp().Set(Gf.Vec3d(*frames[body]))
        body_paths[body] = path_b
        if fixed:
            continue
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(MASSES[body])
        if body.startswith("g") or body in ("anchor", "carrier", "clamp"):  # no colliders: give them an inertia
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(1e-7, 1e-7, 1e-7))
        elif body.startswith("link"):  # a rod (its inertia set, with or without colliders)
            i_rod = MASSES[body] * CHAIN_LINK ** 2 / 12
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(i_rod, i_rod, MASSES[body] * CHAIN_R ** 2 / 2))
        _apply(prim, "PhysxRigidBodyAPI", {
            "physxRigidBody:solverPositionIterationCount": 32,
            "physxRigidBody:solverVelocityIterationCount": 4,
            "physxRigidBody:maxDepenetrationVelocity": 0.3,
            "physxRigidBody:enableCCD": False,
        })

    for i, sh in enumerate(SHAPES[kind]()):
        parent = body_paths[sh.body]
        off = frames.get(sh.body, (0.0, 0.0, 0.0))  # shapes are written in the part's frame; bodies may be offset

        def loc(p, off=off):
            return tuple(a - b for a, b in zip(p, off, strict=True))

        if sh.hull is not None:
            pts = [loc(p) for p in sh.hull]
            if sh.visible:
                geom = UsdGeom.Mesh.Define(stage, f"{parent}/hullvis_{i}")
                faces = _hull_mesh(pts)
                geom.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
                geom.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
                geom.CreateFaceVertexIndicesAttr(Vt.IntArray([v for f in faces for v in f]))
                geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
                UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(looks[sh.look])
            geom = UsdGeom.Mesh.Define(stage, f"{parent}/hull_{i}")
            n = len(pts)
            geom.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
            geom.CreateFaceVertexCountsAttr(Vt.IntArray([3] * (n - 2)))
            geom.CreateFaceVertexIndicesAttr(Vt.IntArray([v for j in range(1, n - 1) for v in (0, j, j + 1)]))
            geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            UsdGeom.Imageable(geom.GetPrim()).MakeInvisible()
        elif sh.profile is not None:
            geom = UsdGeom.Mesh.Define(stage, f"{parent}/prism_{i}")
            pts, counts, indices = _prism_mesh(sh)
            geom.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*loc(p)) for p in pts]))
            geom.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
            geom.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
            geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            geom.CreateDoubleSidedAttr(True)
        elif sh.size is not None:
            geom = UsdGeom.Cube.Define(stage, f"{parent}/box_{i}")
            geom.CreateSizeAttr(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*loc(sh.center)))
            geom.AddScaleOp().Set(Gf.Vec3f(*sh.size))
        elif sh.axis == "S":
            geom = UsdGeom.Sphere.Define(stage, f"{parent}/sphere_{i}")
            geom.CreateRadiusAttr(sh.radius)
            geom.AddTranslateOp().Set(Gf.Vec3d(*loc(sh.center)))
        else:
            geom = UsdGeom.Cylinder.Define(stage, f"{parent}/cyl_{i}")
            geom.CreateRadiusAttr(sh.radius)
            geom.CreateHeightAttr(sh.length)
            geom.CreateAxisAttr(sh.axis)
            geom.AddTranslateOp().Set(Gf.Vec3d(*loc(sh.center)))
        prim = geom.GetPrim()
        if not sh.visible:
            UsdGeom.Imageable(prim).MakeInvisible()
        if sh.hull is None:
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(looks[sh.look])
        if sh.material is None:
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        if sh.profile is not None or sh.hull is not None:
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mats[sh.material], UsdShade.Tokens.strongerThanDescendants,
                                                     "physics")
        _apply(prim, "PhysxCollisionAPI", {"physxCollision:contactOffset": CONTACT_OFFSET,
                                           "physxCollision:restOffset": 0.0})

    if jointed:
        anchor = stage.GetPrimAtPath(body_paths["anchor"])
        UsdPhysics.ArticulationRootAPI.Apply(anchor)
        _apply(anchor, "PhysxArticulationAPI", {
            "physxArticulation:enabledSelfCollisions": False,
            "physxArticulation:solverPositionIterationCount": 64,
            "physxArticulation:solverVelocityIterationCount": 4,
        })
        ident = Gf.Quatf(1, 0, 0, 0)
        if kind == "thread":
            turns = THREAD_RANGE / PITCH
            joints = {  # name: (type, parent, child, axis, lower, upper, stiffness, damping, armature)
                "thread": ("pri", "anchor", "carrier", "Z", -THREAD_RANGE, THREAD_RANGE, 0.0, 0.0, LINK_ARMATURE),
                "clamp": ("pri", "carrier", "clamp", "Z", 0.0, CLAMP_TRAVEL, CLAMP_SPRING, 20.0, LINK_ARMATURE),
                "spin": ("rev", "clamp", "eyebolt", "Z", -360.0 * turns, 360.0 * turns, 0.0, 0.0, EYEBOLT_ARMATURE),
            }
        else:
            joints = {name: ("rev", p, c, ax, lo, hi, k, d, LINK_ARMATURE)
                      for name, (p, c, ax, lo, hi, k, d) in sling_joints().items()}
        for name, (jt, parent, child, ax, lo, hi, k, d, arm) in joints.items():
            joint = (UsdPhysics.RevoluteJoint if jt == "rev" else UsdPhysics.PrismaticJoint).Define(
                stage, f"/Part/joints/{name}")
            joint.CreateBody0Rel().SetTargets([Sdf.Path(body_paths[parent])])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(body_paths[child])])
            joint.CreateAxisAttr(ax)
            joint.CreateLowerLimitAttr(lo)
            joint.CreateUpperLimitAttr(hi)
            # the joint at the child's frame origin, expressed in both bodies' frames
            fp, fc = frames.get(parent, (0.0, 0.0, 0.0)), frames.get(child, (0.0, 0.0, 0.0))
            joint.CreateLocalPos0Attr(Gf.Vec3f(*(c - p for c, p in zip(fc, fp, strict=True))))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            joint.CreateLocalRot0Attr(ident)
            joint.CreateLocalRot1Attr(ident)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular" if jt == "rev" else "linear")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(k)
            drive.CreateDampingAttr(d)
            drive.CreateTargetPositionAttr(0.0)
            _apply(joint.GetPrim(), "PhysxJointAPI", {"physxJoint:armature": arm})
        if kind == "thread":
            spin = stage.GetPrimAtPath("/Part/joints/spin")
            _apply(spin, "PhysxMimicJointAPI:rotZ", {"physxMimicJoint:rotZ:gearing": float(-360.0 / PITCH),
                                                     "physxMimicJoint:rotZ:offset": 0.0})
            spin.CreateRelationship("physxMimicJoint:rotZ:referenceJoint").SetTargets(
                [Sdf.Path("/Part/joints/thread")])
    stage.GetRootLayer().Save()
    return path


def usd_file(kind: str, fixed: bool = False) -> str:
    global _USD_DIR
    if _USD_DIR is None:
        _USD_DIR = Path(tempfile.mkdtemp(prefix="eyebolt_lift_parts_"))
    path = _USD_DIR / f"{kind}{'_fixed' if fixed else ''}.usda"
    if not path.exists():
        write_usd(kind, path, fixed)
    return str(path)


_THREADS: dict[str, str] = {}  # loose eyebolt name -> its twin's name
_HOLES: list[tuple[float, float, float, float]] = []  # (x, y, top z, thread phase yaw), env frame
_HOLE_PHASES: list[float] = []  # set by the task before spawning: per hole, the eye's yaw at shoulder contact
_SLINGS: list[str] = []


def set_hole_phases(phases) -> None:
    """The eye's yaw (rad, world) at shoulder contact, one per hole in spawn order (the task's choice)."""
    _HOLE_PHASES[:] = list(phases)


def arena_object(spec: Part):
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    class _Thread(Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            cfg.actuators = {
                "thread": ImplicitActuatorCfg(joint_names_expr=["thread"], stiffness=0.0, damping=0.0,
                                              armature=LINK_ARMATURE),
                "clamp": ImplicitActuatorCfg(joint_names_expr=["clamp"], stiffness=CLAMP_SPRING, damping=20.0,
                                             armature=LINK_ARMATURE),
                "spin": ImplicitActuatorCfg(joint_names_expr=["spin"], stiffness=0.0, damping=0.0,
                                            armature=EYEBOLT_ARMATURE, friction=FREE_TORQUE,
                                            dynamic_friction=FREE_TORQUE),
            }
            cfg.init_state.joint_pos = {"thread": 0.0, "clamp": 0.0, "spin": 0.0}
            return cfg

    class _Sling(Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            cfg.actuators = {
                "chain": ImplicitActuatorCfg(joint_names_expr=["c.*"], stiffness=0.0, damping=CHAIN_DAMPING,
                                             armature=LINK_ARMATURE),
                "latch": ImplicitActuatorCfg(joint_names_expr=["latch"], stiffness=LATCH_SPRING,
                                             damping=LATCH_DAMPING, armature=LINK_ARMATURE),
            }
            cfg.init_state.joint_pos = dict(SLING_INIT) if SLING_INIT else {".*": 0.0}
            return cfg

    pose = Pose(position_xyz=tuple(spec.pos), rotation_xyzw=part_quat_xyzw(spec))
    if spec.kind in ("gantry", "stand"):
        cfg = sim_utils.UsdFileCfg(usd_path=usd_file(spec.kind, fixed=True))
        return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg, initial_pose=pose)
    if spec.kind == "housing":
        c, s = math.cos(spec.yaw), math.sin(spec.yaw)
        for x, y in HOUSING_HOLES:
            k = len(_HOLES)
            phase = _HOLE_PHASES[k] if k < len(_HOLE_PHASES) else 0.0
            _HOLES.append((spec.pos[0] + c * x - s * y, spec.pos[1] + s * x + c * y, spec.pos[2], phase))
        cfg = sim_utils.UsdFileCfg(usd_path=usd_file("housing", fixed=True))
        return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg, initial_pose=pose)
    if spec.kind in ("thread", "sling"):
        if spec.kind == "thread":
            _THREADS[spec.name[: -len("_thread")]] = spec.name
        else:
            _SLINGS.append(spec.name)
        cfg = sim_utils.UsdFileCfg(usd_path=usd_file(spec.kind))
        cfg.articulation_props = sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=64, solver_velocity_iteration_count=4,
            fix_root_link=True)
        cls = _Thread if spec.kind == "thread" else _Sling
        return cls(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=cfg, initial_pose=pose)
    fixed = spec.mass is None
    cfg = sim_utils.UsdFileCfg(usd_path=usd_file(spec.kind, fixed))
    return Object(name=spec.name, object_type=ObjectType.BASE if fixed else ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=pose)


# -- the thread swap ------------------------------------------------------------------------------------------------


def _rot(quat_xyzw):
    import numpy as np

    x, y, z, w = quat_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _yaw_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


@dataclass
class _Track:
    engaged: bool = False
    hole: int = -1
    turned: float = 0.0
    contact: float | None = None
    pressing: int = 0
    pressing_from: float = 0.0
    last_yaw: float | None = None
    swaps: int = 0
    thread0: float = 0.0  # the thread slider's position when it caught (turns count from there)
    snap: float = 0.0  # m: how far the depth snapped onto the helix at the catch
    history: list = field(default_factory=list)


_TRACK: dict[str, _Track] = {}
_EPISODE_STEP: list = [None]


def _np(x):
    import numpy as np

    return (x.torch if hasattr(x, "torch") else x)[0].detach().cpu().numpy().astype(np.float64)


def _tensor(base, rows):
    import torch

    return torch.tensor(rows, dtype=torch.float32, device=base.device)


def _set_rigid(base, name, pos_env, quat_xyzw):
    obj = base.scene.rigid_objects[name]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    p = [float(v) for v in (pos_env + origin)]
    obj.write_root_pose_to_sim_index(root_pose=_tensor(base, [p + list(quat_xyzw)]), env_ids=[0])
    obj.write_root_velocity_to_sim_index(root_velocity=_tensor(base, [[0.0] * 6]), env_ids=[0])


def _set_twin(base, name, pos_env, quat_xyzw, thread: float = 0.0, spin: float = 0.0):
    """Place a twin's anchor and set its joints (thread m, spin rad; consistent with the mimic coupling), at rest."""
    art = base.scene.articulations[name]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    p = [float(v) for v in (pos_env + origin)]
    art.write_root_pose_to_sim_index(root_pose=_tensor(base, [p + list(quat_xyzw)]), env_ids=[0])
    names = list(art.joint_names)
    pos = [[0.0] * len(names)]
    pos[0][names.index("thread")] = thread
    pos[0][names.index("spin")] = spin
    art.write_joint_position_to_sim_index(position=_tensor(base, pos), env_ids=[0])
    art.write_joint_velocity_to_sim_index(velocity=_tensor(base, [[0.0] * len(names)]), env_ids=[0])


def twin_mechanism(base, twin: str) -> dict:
    art = base.scene.articulations[twin]
    q = _np(art.data.joint_pos)
    names = list(art.joint_names)
    thread, clamp, spin = (float(q[names.index(j)]) for j in ("thread", "clamp", "spin"))
    return {"thread": thread, "clamp": clamp, "spin": spin, "clamp_force": CLAMP_SPRING * max(0.0, clamp)}


def _body_pose(base, art_name: str, body: str):
    art = base.scene.articulations[art_name]
    i = list(art.body_names).index(body)
    pos = (art.data.body_pos_w.torch if hasattr(art.data.body_pos_w, "torch") else art.data.body_pos_w)[0, i]
    quat = (art.data.body_quat_w.torch if hasattr(art.data.body_quat_w, "torch") else art.data.body_quat_w)[0, i]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    return pos.detach().cpu().numpy().astype(float) - origin, quat.detach().cpu().numpy().astype(float)


def _body_vel(base, art_name: str, body: str):
    art = base.scene.articulations[art_name]
    i = list(art.body_names).index(body)
    lin = art.data.body_lin_vel_w.torch if hasattr(art.data.body_lin_vel_w, "torch") else art.data.body_lin_vel_w
    ang = art.data.body_ang_vel_w.torch if hasattr(art.data.body_ang_vel_w, "torch") else art.data.body_ang_vel_w
    return lin[0, i].detach().cpu().numpy().astype(float), ang[0, i].detach().cpu().numpy().astype(float)


def park_pos(base, name: str):
    import numpy as np

    if name in _THREADS.values():
        return np.array(_PARKED[name])
    p = np.array(_PARKED[_THREADS[name]])
    return p + np.array([0.0, 0.05, -PARK[2] + 0.01])


_PARKED: dict[str, tuple] = {}


def helix_joints(phase: float, yaw: float, tip_z: float, anchor_z: float):
    """The twin's (thread, spin) that keep an eyebolt at ``yaw`` on the hole's helix nearest ``tip_z``: the anchor (at
    ``anchor_z``, turned to ``phase``) is where the tip is when the shoulder sits on the face, at the phase's angle."""
    spin0 = (yaw - phase + math.pi) % (2 * math.pi) - math.pi  # the angle within one turn
    want = tip_z - anchor_z  # the thread slider's position for this depth (positive: above the seat)
    # thread = spin / 2pi * PITCH; choose the whole turns k that put it nearest the depth
    k = round((want / PITCH * 2 * math.pi - spin0) / (2 * math.pi))
    spin = spin0 + 2 * math.pi * k
    return spin / (2 * math.pi) * PITCH, spin


def update_threads(base) -> None:
    """Catch or release each eyebolt's thread (once per control step)."""
    import numpy as np

    step = getattr(base, "episode_length_buf", None)
    step = None if step is None else int(step[0])
    if step == 0 or (_EPISODE_STEP[0] is not None and step is not None and step < _EPISODE_STEP[0]):
        _TRACK.clear()
    _EPISODE_STEP[0] = step
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for bolt, twin in _THREADS.items():
        if twin not in _PARKED:
            _PARKED[twin] = tuple(_np(base.scene.articulations[twin].data.root_pose_w)[:3] - origin)
        tr = _TRACK.setdefault(bolt, _Track())
        if not tr.engaged:
            pose = _np(base.scene.rigid_objects[bolt].data.root_pose_w)
            tip, rot = pose[:3] - origin, _rot(pose[3:])
            tilt = math.acos(min(1.0, max(-1.0, rot[2, 2])))
            seated = None
            r_ok = ENGAGE_R + (ENGAGE_R_HYST if tr.turned > 0.0 else 0.0)
            for i, (hx, hy, hz, _) in enumerate(_HOLES):
                if (math.hypot(tip[0] - hx, tip[1] - hy) < r_ok
                        and hz - SEAT_DEPTH - ENGAGE_DZ < tip[2] < hz - ENGAGE_MIN_DEPTH
                        and tilt < ENGAGE_TILT):
                    seated = i
            if seated is None:
                tr.turned, tr.last_yaw = 0.0, None
                continue
            yaw = math.atan2(rot[1, 0], rot[0, 0])
            if tr.last_yaw is not None:
                d = (yaw - tr.last_yaw + math.pi) % (2 * math.pi) - math.pi
                tr.turned = max(0.0, tr.turned - d)
            tr.last_yaw = yaw
            if tr.turned < START_ANGLE:
                continue
            # caught: the twin on the hole's helix, keeping the eyebolt's angle; its depth snaps (< half a pitch)
            hx, hy, hz, phase = _HOLES[seated]
            anchor_z = hz - THREAD_LEN
            thread, spin = helix_joints(phase, yaw, tip[2], anchor_z)
            _set_twin(base, twin, np.array([hx, hy, anchor_z]), _yaw_quat(phase), thread=thread, spin=spin)
            _set_rigid(base, bolt, park_pos(base, bolt), (0.0, 0.0, 0.0, 1.0))
            tr.engaged, tr.hole, tr.turned, tr.last_yaw = True, seated, 0.0, None
            tr.thread0, tr.snap = thread, (anchor_z + thread) - tip[2]
            tr.swaps += 1
        else:
            m = twin_mechanism(base, twin)
            turns = -(m["thread"] - tr.thread0) / PITCH
            if m["clamp_force"] >= CONTACT_CLAMP:
                if tr.pressing == 0:
                    tr.pressing_from = turns
                tr.pressing += 1
            else:
                tr.pressing = 0
            if tr.contact is None and tr.pressing >= CONTACT_STEPS:
                tr.contact = tr.pressing_from
            elif tr.contact is not None and turns < tr.contact - 0.05 and m["clamp_force"] < CONTACT_CLAMP:
                tr.contact = None
            if turns > -BACKOUT_TURNS:
                continue
            pos, quat = _body_pose(base, twin, "eyebolt")
            _set_rigid(base, bolt, pos + np.array([0.0, 0.0, 0.05 * MM]), quat)
            _set_twin(base, twin, np.array(_PARKED[twin]), (0.0, 0.0, 0.0, 1.0))
            tr.engaged, tr.hole, tr.contact, tr.pressing = False, -1, None, 0
            tr.swaps += 1


def _corners(kind, pos, rot):
    import numpy as np

    lo, hi = bounds(kind)
    return np.array([[(lo, hi)[(i >> k) & 1][k] for k in range(3)] for i in range(8)]) @ rot.T + pos


def _bolt_state(base, bolt: str, rigid_state) -> BoltState:
    import numpy as np

    tr = _TRACK.get(bolt)
    if tr is None or not tr.engaged:
        return BoltState(**{f: getattr(rigid_state, f) for f in ("pos", "quat_xyzw", "lin_vel", "ang_vel", "low",
                                                                 "high")})
    twin = _THREADS[bolt]
    pos, quat = _body_pose(base, twin, "eyebolt")
    rot = _rot(quat)
    lin, ang = _body_vel(base, twin, "eyebolt")
    m = twin_mechanism(base, twin)
    turns = -(m["thread"] - tr.thread0) / PITCH
    corners = _corners("eyebolt", pos, rot)
    hz = _HOLES[tr.hole][2]
    gap = float((pos + rot @ np.array([0.0, 0.0, THREAD_LEN]))[2] - hz)  # the shoulder's underside above the face
    return BoltState(pos=rot @ np.array(local_center("eyebolt")) + pos, quat_xyzw=quat, lin_vel=lin, ang_vel=ang,
                     low=corners.min(axis=0), high=corners.max(axis=0), engaged=True, hole=tr.hole, turns=turns,
                     clamp_force=m["clamp_force"],
                     past_contact=0.0 if tr.contact is None else turns - tr.contact, gap=gap,
                     touching=tr.contact is not None)


def _twin_state(base, twin: str):
    import numpy as np

    pose = _np(base.scene.articulations[twin].data.root_pose_w)
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    p = pose[:3] - origin
    z = np.zeros(3)
    return task_file.ObjectState(pos=p, quat_xyzw=pose[3:], lin_vel=z, ang_vel=z, low=p, high=p)


def _sling_state(base, name: str) -> SlingState:
    import numpy as np

    pos, quat = _body_pose(base, name, "hook")
    lin, ang = _body_vel(base, name, "hook")
    art = base.scene.articulations[name]
    q = _np(art.data.joint_pos)
    latch = float(q[list(art.joint_names).index("latch")])
    apos, _ = _body_pose(base, name, "anchor")
    corners = _corners_hook(pos, _rot(quat))
    hand = float("inf")
    for art_name, other in base.scene.articulations.items():  # the robot's fingers (yam_arena: *_robot, tip_*)
        if art_name in _SLINGS or art_name in _THREADS.values():
            continue
        for b in other.body_names:
            if b.startswith("tip"):
                p, q = _body_pose(base, art_name, b)
                pts = finger_points(b) @ _rot(q).T + p  # the finger's collision surface (its body's origin sits
                # 7-10 cm from the pads)
                d = np.linalg.norm(corners[:, None, :] - pts[None, :, :], axis=2)
                hand = min(hand, float(d.min()))
    return SlingState(pos=pos, quat_xyzw=quat, lin_vel=lin, ang_vel=ang, low=corners.min(axis=0),
                      high=corners.max(axis=0), latch=latch, anchor=tuple(float(v) for v in apos), hand_dist=hand)


_FINGER_PTS: dict = {}


def finger_points(body: str):
    """Points on a gripper finger's collision meshes (vertices, thinned to <= 400), in its body's frame, read from
    yam_arena's robot description."""
    import struct
    import xml.etree.ElementTree as ET

    import numpy as np

    if body in _FINGER_PTS:
        return _FINGER_PTS[body]
    import yam_arena

    root = Path(yam_arena.__file__).resolve().parent / "assets" / "yam"
    pts = []
    links = [lk for f in ("yam_arm.urdf", "yam.urdf") for lk in ET.parse(root / f).getroot().iter("link")
             if lk.get("name") == body and lk.find("collision") is not None]
    for link in links[:1]:
        for col in link.iter("collision"):
            org = col.find("origin")
            xyz = np.array([float(v) for v in (org.get("xyz", "0 0 0") if org is not None else "0 0 0").split()])
            mesh = col.find("geometry/mesh")
            data = (root / mesh.get("filename")).read_bytes()
            if data[:5] == b"solid":  # ASCII STL
                v = [[float(x) for x in ln.split()[1:4]] for ln in data.decode(errors="ignore").splitlines()
                     if ln.strip().startswith("vertex")]
            else:
                n = struct.unpack("<I", data[80:84])[0]
                v = [struct.unpack("<3f", data[84 + 50 * i + 12 + 12 * k: 84 + 50 * i + 24 + 12 * k])
                     for i in range(n) for k in range(3)]
            pts.append(np.asarray(v, dtype=float) + xyz)
    allp = np.unique(np.round(np.concatenate(pts), 5), axis=0)
    if len(allp) > 400:
        allp = allp[np.linspace(0, len(allp) - 1, 400).astype(int)]
    _FINGER_PTS[body] = allp
    return allp


def _corners_hook(pos, rot):
    import numpy as np

    pts = np.array([p for sh in hook_shapes_local() for p in sh.hull])
    return pts @ rot.T + pos


def hooked(bolt_state, sling_state, tol: float = 0.0) -> bool:
    """The hook through the eye: some point of the eye's centreline inside the hook's bowl, below the closed latch, in
    the hook's plane (the hook slides along the eye to where the chain pulls; only its top section was checked first,
    hook bench run 5)."""
    import numpy as np

    rb = _rot(bolt_state.quat_xyzw)
    tip = np.asarray(bolt_state.pos) - rb @ np.array(local_center("eyebolt"))
    for k in range(48):
        t = 2 * math.pi * k / 48
        p = tip + rb @ np.array([RING_RC * math.sin(t), 0.0, RING_Z - RING_RC * math.cos(t)])
        if abs(in_hook(p, sling_state)[1]) < RING_RS and hooked_point(p, sling_state, tol):
            return True
    return False


def in_hook(point, sling_state):
    """A point in the hook's frame (x toward the shank's side, z up the shank; origin at its top joint)."""
    import numpy as np

    rh = _rot(sling_state.quat_xyzw)
    return rh.T @ (np.asarray(point, dtype=float) - np.asarray(sling_state.pos))


def hooked_point(point, sling_state, tol: float = 0.0) -> bool:
    """A bar's section centre (``point``, world) inside the hook's bowl, below the closed latch."""
    p = in_hook(point, sling_state)
    bx, bz = HOOK_BOWL_C
    inside_bowl = math.hypot(p[0] - bx, max(0.0, bz - p[2])) < HOOK_BOWL_R - HOOK_W + tol and \
        bx - HOOK_BOWL_R + HOOK_W - tol < p[0] < -HOOK_W + tol
    (hx, hz), (ex, ez) = LATCH_HINGE, LATCH_END  # below the closed latch's line (by 1 mm)
    below_latch = p[2] < hz + (ez - hz) * min(1.0, max(0.0, (p[0] - hx) / (ex - hx))) - 1.0 * MM
    return bool(inside_bowl and below_latch and abs(p[1]) < RING_RC and sling_state.latch > -math.radians(10.0))


# -- the hook into yam_arena.scene ---------------------------------------------------------------------------------------


def _patch(scene_module) -> None:
    original = getattr(scene_module, "task_object", None)
    original_states = getattr(scene_module, "object_states", None)
    for fn, params in ((original, ["spec"]), (original_states, ["env", "names"])):
        if not callable(fn) or list(inspect.signature(fn).parameters)[: len(params)] != params:
            raise RuntimeError(
                "el_parts: yam_arena.scene no longer has task_object(spec) and object_states(env, names); "
                "this task's parts hook needs updating for this yam_arena version"
            )
    if getattr(original, "_spawns_el_parts", False):
        return

    def task_object(spec):
        return arena_object(spec) if isinstance(spec, Part) else original(spec)

    def object_states(env, names):
        base = env.unwrapped
        update_threads(base)
        special = {n for n in names if n in _THREADS.values() or n in _SLINGS}
        out = original_states(env, [n for n in names if n not in special])
        for n in names:
            if n in _THREADS.values():
                out[n] = _twin_state(base, n)
            elif n in _SLINGS:
                out[n] = _sling_state(base, n)
            elif n in _THREADS:
                out[n] = _bolt_state(base, n, out[n])
        return {n: out[n] for n in names}

    task_object._spawns_el_parts = True
    task_object.__doc__ = original.__doc__
    object_states.__doc__ = original_states.__doc__
    scene_module.task_object = task_object
    scene_module.object_states = object_states


class _SceneHook(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is None or spec.loader is None:
            return spec
        loader = spec.loader
        exec_module = loader.exec_module

        def exec_and_patch(module):
            exec_module(module)
            _patch(module)

        loader.exec_module = exec_and_patch
        return spec


def install() -> None:
    """Make yam_arena spawn this module's parts (call once, before ``yam_arena.scene`` is imported)."""
    if "yam_arena.scene" in sys.modules:
        _patch(sys.modules["yam_arena.scene"])
        return
    if not any(isinstance(f, _SceneHook) for f in sys.meta_path):
        sys.meta_path.insert(0, _SceneHook())
